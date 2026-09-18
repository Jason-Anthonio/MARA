#include <Wire.h>
#include <util/atomic.h>

const int PWM_PIN = 3;
const int PWM_PIN_2 = 19;
const int AS5600_ADDR = 0x36;

volatile unsigned long riseTime = 0;
volatile unsigned long highTime = 0;
volatile unsigned long totalPeriod = 0;
volatile unsigned long riseTime2 = 0;
volatile unsigned long highTime2 = 0;
volatile unsigned long totalPeriod2 = 0;

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}

void configureAS5600(uint8_t bus) {
  TCA9548A(bus);

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x08);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 1);
  if (Wire.available() == 0) {
    Serial.println("ERROR: AS5600 not found on I2C bus!");
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
  configureAS5600(2);
  configureAS5600(3);

  //setup the interrupt using a pwm interrupt pin
  pinMode(PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), catchWave, CHANGE);
  pinMode(PWM_PIN_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN_2), catchWave2, CHANGE);
}

//interrupt function
void catchWave() {
  unsigned long rightNow = micros(); // microseconds

  if (digitalRead(PWM_PIN) == HIGH) {
    totalPeriod = rightNow - riseTime; // How long since the last HIGH
    riseTime = rightNow;               // Reset the timer

  } else {
    highTime = rightNow - riseTime;    // HIGH duration
  }
}

void catchWave2() {
  unsigned long rightNow = micros();

  if (digitalRead(PWM_PIN_2) == HIGH) {
    totalPeriod2 = rightNow - riseTime2;
    riseTime2 = rightNow;
  } else {
    highTime2 = rightNow - riseTime2;
  }
}

void loop() {
  unsigned long myHigh, myPeriod, myHigh2, myPeriod2;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    myHigh = highTime;
    myPeriod = totalPeriod;
    myHigh2 = highTime2;
    myPeriod2 = totalPeriod2;
  }  

  if (myPeriod > 0) {
    long clockTicks = (myHigh * 4351) / myPeriod;
    int rawAngle = clockTicks - 128;
    if (rawAngle < 0) rawAngle = 0;
    if (rawAngle > 4095) rawAngle = 4095;
    float degrees = rawAngle * (360.0 / 4096.0);

    Serial.print("AS5600 bus 2 angle: ");
    Serial.println(degrees);
  }

  if (myPeriod2 > 0) {
    long clockTicks2 = (myHigh2 * 4351) / myPeriod2;
    int rawAngle2 = clockTicks2 - 128;
    if (rawAngle2 < 0) rawAngle2 = 0;
    if (rawAngle2 > 4095) rawAngle2 = 4095;
    float degrees2 = rawAngle2 * (360.0 / 4096.0);

    Serial.print("AS5600 bus 3 angle: ");
    Serial.println(degrees2);
  }

  delay(100);
}