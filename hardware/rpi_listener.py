#Run on Rpi onboard Roomi
import socket, serial, time, sys, threading
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw

UDP_PORT   = 5005
TX_HZ      = 50
BUF_SIZE   = 256

PICO_PORTS = ["/dev/ttyACM1", "/dev/ttyACM0"] #The two arm picos
BAUD_PICO  = 115_200

SERVO_PORT    = "/dev/ttyACM2" #Gripper USB to TTL port
SERVO_BAUD    = 1_000_000
SERVO_ID      = 6
ANGLE_ON_DEG  = 130
ANGLE_OFF_DEG = 215

CTRL_PORT  = "/dev/ttyACM3" #Wheels pico port
CTRL_BAUD  = 115_200

def open_serial(path, baud):
    try:
        ser = serial.Serial(path, baud, timeout=0)
        print(f"✓ Opened {path} @ {baud}")
        time.sleep(0.05)
        return ser
    except serial.SerialException as e:
        sys.exit(f"Cannot open {path}: {e}")


def _checksum(b): return (~(sum(b) & 0xFF)) & 0xFF
def write_servo_pos(ser, deg):
    pos = int((deg % 360) * 4096 / 360)
    p   = [0x2A, pos & 0xFF, pos >> 8 & 0xFF, 0, 0, 0, 0]
    f   = [0xFF, 0xFF, SERVO_ID, len(p)+2, 0x03] + p + [_checksum(p + [0x03, SERVO_ID])]
    ser.write(bytearray(f))


def static_eyes():
    devs = [ssd1306(i2c(port=1, address=addr)) for addr in (0x3C, 0x3D)]
    for dev in devs:
        w, h = dev.width, dev.height
        img  = Image.new("1", (w, h))
        d    = ImageDraw.Draw(img)
        margin, radius = 10, 20
        d.rounded_rectangle([(margin, margin),
                             (w - margin, h)],
                            radius=radius, fill=255)
        dev.display(img)



def main():
    # open serial ports
    if len(PICO_PORTS) != 2:
        sys.exit("PICO_PORTS must list exactly two entries.")
    pico0 = open_serial(PICO_PORTS[0], BAUD_PICO)
    pico1 = open_serial(PICO_PORTS[1], BAUD_PICO)
    servo = open_serial(SERVO_PORT, SERVO_BAUD)
    ctrl  = open_serial(CTRL_PORT,  CTRL_BAUD)

    # draw OLEDs once (no thread needed)
    static_eyes()

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))
    sock.settimeout(0.02)

    latest = [0.0] * 7
    period = 1.0 / TX_HZ
    next_t = time.time()

    try:
        while True:
            try:
                payload, _ = sock.recvfrom(BUF_SIZE)
                vals = [float(x) for x in payload.decode().strip().split(",")]
                if len(vals) == 7:
                    latest = vals
            except socket.timeout:
                pass
            except ValueError:
                pass

            now = time.time()
            if now >= next_t:
                v1, v2, v3, v4, k1, k2, flag = latest

                pico0.write(f"1t{v4}\n0t{v3}\n".encode())
                pico1.write(f"1t{v2}\n0t{v1}\n".encode())
                write_servo_pos(servo,
                                ANGLE_ON_DEG if int(flag) == 1 else ANGLE_OFF_DEG)

                if   (k1, k2) == (1,  1): cmd = ("0b", "1f")
                elif (k1, k2) == (0,  0): cmd = ("0s", "1s")
                elif (k1, k2) == (-1, -1): cmd = ("0f", "1b")
                elif (k1, k2) == (-1,  1): cmd = ("0f", "1f")
                elif (k1, k2) == (1, -1):  cmd = ("0b", "1b")
                else: cmd = None

                if cmd:
                    ctrl.write((cmd[0] + "\n").encode())
                    ctrl.write((cmd[1] + "\n").encode())

                print(f"[{time.strftime('%H:%M:%S')}] "
                      f"P0→[{v1:.1f},{v2:.1f}]  "
                      f"P1→[{v3:.1f},{v4:.1f}]  "
                      f"Servo→{('ON' if int(flag) else 'OFF')}  "
                      f"Ctrl→{cmd}")

                next_t += period

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nExiting…")
    finally:
        for s in (pico0, pico1, servo, ctrl):
            s.close()
        sock.close()

if __name__ == "__main__":
    main()
