
#include <Arduino.h>

constexpr uint8_t M0_STEP_PIN = 4;
constexpr uint8_t M0_DIR_PIN  = 5;
constexpr uint8_t M0_EN_PIN   = 6;

constexpr uint8_t M1_STEP_PIN =  7;
constexpr uint8_t M1_DIR_PIN  = 8;
constexpr uint8_t M1_EN_PIN   = 9;

constexpr uint32_t STEPS_PER_REV = 200;   // 1.8 ° motor
constexpr uint32_t MICROSTEPS    = 8;     // driver set to x8
constexpr float    RPM           = 160.0;  // chosen speed
constexpr uint32_t STEP_FREQ_HZ  =
          STEPS_PER_REV * MICROSTEPS * RPM / 60.0;          // ≈1600 Hz
constexpr uint32_t STEP_PERIOD_US = 1'000'000UL / STEP_FREQ_HZ;

struct Motor {
  const uint8_t stepPin, dirPin, enPin;
  volatile bool running  = false;
  volatile uint32_t next = 0;
};

Motor m0{M0_STEP_PIN, M0_DIR_PIN, M0_EN_PIN};
Motor m1{M1_STEP_PIN, M1_DIR_PIN, M1_EN_PIN};

void startMotor(Motor& m, bool forward) {
  digitalWrite(m.dirPin, forward ? HIGH : LOW);
  digitalWrite(m.enPin,  LOW);      // enable driver
  m.running = true;
  m.next    = micros();             // step immediately
}

void stopMotor(Motor& m) {
  m.running = false;
  digitalWrite(m.enPin, HIGH);      // disable driver
}

void pulseStepper(Motor& m) {
  if (!m.running) return;
  uint32_t now = micros();
  if ((int32_t)(now - m.next) >= 0) {
    digitalWrite(m.stepPin, HIGH);
    delayMicroseconds(2);           // ≥1 µs HIGH
    digitalWrite(m.stepPin, LOW);
    m.next += STEP_PERIOD_US;
  }
}

void handleSerial() {
  while (Serial.available()) {
    char id = Serial.read();                  // '0' or '1'
    if (id != '0' && id != '1') continue;
    while (!Serial.available()) {}            // wait for verb
    char cmd = Serial.read();                 // f / b / s

    Motor& m = (id == '0') ? m0 : m1;
    switch (cmd) {
      case 'f': startMotor(m, true ); break;
      case 'b': startMotor(m, false); break;
      case 's': stopMotor (m);        break;
      default : /* ignore */          break;
    }
  }
}

void setupMotorPins(Motor& m) {
  pinMode(m.stepPin, OUTPUT);
  pinMode(m.dirPin , OUTPUT);
  pinMode(m.enPin  , OUTPUT);
  digitalWrite(m.stepPin, LOW);
  digitalWrite(m.dirPin , LOW);
  digitalWrite(m.enPin  , HIGH);   // disabled at boot
}


void setup() {
  Serial.begin(115200);
  setupMotorPins(m0);
  setupMotorPins(m1);
  Serial.println(F("Dual‑TMC2209 constant‑speed demo ready (8 µsteps)"));
} 
void loop() {
  handleSerial();
  pulseStepper(m0);
  pulseStepper(m1);
}
