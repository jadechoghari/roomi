"""
Laptopside teleop controller, UDP communication w/ Raspberry Pi
────────────────────
Packet: J1 J2 J3 J4 WheelL WheelR Gripper
Dependencies:
    pip install --break-system-packages pyserial keyboard
On Linux you’ll need to run with sudo (or grant your user access to /dev/input).
"""

import socket, serial, time, keyboard, sys

RPI_IP        = "10.10.126.25"
UDP_PORT      = 5005

SERVO_PORT    = "COM20"        # e.g. "/dev/ttyUSB0"
SERVO_BAUD    = 1_000_000
SERVO_IDS     = [1, 2, 3, 4]

TX_HZ         = 100            # UDP packet rate

HEADER              = 0xFF
INSTR_READ           = 0x02
REG_PRESENT_POS_L    = 0x38
RAW_FULL_SCALE       = 4096
ANGLE_FULL_DEG       = 360.0

def raw_to_degrees(raw):
    return (raw % RAW_FULL_SCALE) * (ANGLE_FULL_DEG / RAW_FULL_SCALE)

class FeetechBus:
    def __init__(self, port, baud, timeout=0.001):
        self.ser = serial.Serial(port, baudrate=baud,
                                 timeout=timeout, write_timeout=timeout)
        time.sleep(0.05)

    def close(self):
        self.ser.close()

    @staticmethod
    def _checksum(body_bytes):
        return (~(sum(body_bytes) & 0xFF)) & 0xFF

    def _flush_input(self):
        self.ser.reset_input_buffer()

    def tx_packet(self, sid, instr, params):
        length = len(params) + 2          # INSTR + CHK
        body   = [sid, length, instr] + params
        frame  = bytes([HEADER, HEADER] + body + [self._checksum(body)])
        self.ser.write(frame)

    def _read_status(self, expect_id, timeout=0.01):
        start = time.time()
        buf   = bytearray()
        while time.time() - start < timeout:
            if self.ser.in_waiting:
                buf += self.ser.read(self.ser.in_waiting)
                while len(buf) >= 4:
                    if buf[0] != HEADER or buf[1] != HEADER:
                        buf.pop(0); continue
                    if len(buf) < 4: break
                    length = buf[3]
                    if len(buf) < 4 + length: break
                    frame = bytes(buf[:4+length]); del buf[:4+length]
                    if self._checksum(list(frame[2:-1])) != frame[-1]:
                        continue
                    sid   = frame[2]
                    err   = frame[4]
                    param = frame[5:-1]
                    if sid == expect_id and err == 0:
                        return param
            else:
                time.sleep(0.0003)
        return None

    def read_reg(self, sid, addr, size=2):
        self._flush_input()
        self.tx_packet(sid, INSTR_READ, [addr, size])
        res = self._read_status(sid)
        return res if res and len(res) == size else None

def get_servo_angles(bus, ids):
    angles = []
    for sid in ids:
        pos = bus.read_reg(sid, REG_PRESENT_POS_L, 2)
        if pos:
            raw = pos[0] | (pos[1] << 8)
            angles.append(round(raw_to_degrees(raw), 2))
        else:
            angles.append(0.0)       # fallback
    while len(angles) < 4:
        angles.append(0.0)
    return angles[:4]


def arrow_vector():
    """Returns (v5, v6) given arrow‑key state."""
    if keyboard.is_pressed('up'):
        return -1, -1
    if keyboard.is_pressed('down'):
        return 1, 1
    if keyboard.is_pressed('right'):
        return 1, -1
    if keyboard.is_pressed('left'):
        return -1, 1
    return 0, 0


def main():
    try:
        bus  = FeetechBus(SERVO_PORT, SERVO_BAUD)
    except serial.SerialException as e:
        sys.exit(f"Cannot open servo port {SERVO_PORT}: {e}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    space_flag    = 0
    space_latched = False     # for edge detect
    period        = 1.0 / TX_HZ

    print(f"Sending UDP to {RPI_IP}:{UDP_PORT} @ {TX_HZ} Hz – Ctrl‑C to quit")
    try:
        while True:
            t0      = time.time()
            v1, v2, v3, v4 = get_servo_angles(bus, SERVO_IDS)

            if v1 > 180:
                v1 = -(360 - v1)
            v1 = 40 + v1*15

            v2 = -((v2-144)*15)

            if v3 < 180:
                v3 = 360 + v3
            v3 = 200 + (v3-245.65)*51

            v4 = -1900 - (v4 - 180)*27
            v5, v6        = arrow_vector()

            # Space‑bar toggle (edge‑trigger)
            if keyboard.is_pressed('space'):
                if not space_latched:
                    space_flag ^= 1
                    space_latched = True
            else:
                space_latched = False

            payload = f"{v1},{v2},{v3},{v4},{v5},{v6},{space_flag}"
            sock.sendto(payload.encode(), (RPI_IP, UDP_PORT))
            # Uncomment for debug:
            print(payload)

            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\nExiting…")
    finally:
        bus.close()
        sock.close()


if __name__ == "__main__":
    main()
