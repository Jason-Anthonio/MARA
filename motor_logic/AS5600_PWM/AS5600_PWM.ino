#include <Wire.h>
#include <util/atomic.h>

const int PWM_PIN = 3;
const int PWM_PIN_2 = 2;
const int AS5600_ADDR = 0x36;

struct PWMData {
  volatile unsigned long riseTime = 0;
  volatile unsigned long highTime = 0;
  volatile unsigned long period = 0;
};

PWMData pwm1;
PWMData pwm2;

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  
  Wire.write(1 << bus);          
  Wire.endTransmission();
  Serial.print("Selected mux channel ");
  Serial.println(bus);
}

void configureAS5600(uint8_t bus) {
  TCA9548A(bus);

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x08);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 1);
  if (Wire.available() == 0) {
    Serial.println("AS5600 not found on I2C bus");
    while (1);
  }
  uint8_t currentSettings = Wire.read();

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x08);
  Wire.write((currentSettings & 0b00001111) | 0b11100000);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  configureAS5600(6);
  configureAS5600(7);

  pinMode(PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), catchWave, CHANGE);
  pinMode(PWM_PIN_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN_2), catchWave2, CHANGE);
}

void capturePWM(uint8_t pin, PWMData& data) {
  unsigned long rightNow = micros();

  if (digitalRead(pin) == HIGH) {
    data.period = rightNow - data.riseTime;
    data.riseTime = rightNow;
  } else {
    data.highTime = rightNow - data.riseTime;
  }
}

void catchWave() {
  capturePWM(PWM_PIN, pwm1);
}

void catchWave2() {
  capturePWM(PWM_PIN_2, pwm2);
}

float calculateAngle(unsigned long highTime, unsigned long period) {
  if (period <= 0) return -1.0;

  long clockTicks = (highTime * 4351UL) / period;
  long rawAngle = clockTicks - 128;
  if (rawAngle < 0) rawAngle = 0;
  if (rawAngle > 4095) rawAngle = 4095;

  return rawAngle * (360.0 / 4096.0);
}

void loop() {
  unsigned long high1, period1, high2, period2;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    high1 = pwm1.highTime;
    period1 = pwm1.period;
    high2 = pwm2.highTime;
    period2 = pwm2.period;
  }

  float angle1 = calculateAngle(high1, period1);
  if (angle1 >= 0) {
    Serial.print("AS5600 bus 6 angle: ");
    Serial.println(angle1);
  }

  float angle2 = calculateAngle(high2, period2);
  if (angle2 >= 0) {
    Serial.print("AS5600 bus 7 angle: ");
    Serial.println(angle2);
  }

  delay(100);
}