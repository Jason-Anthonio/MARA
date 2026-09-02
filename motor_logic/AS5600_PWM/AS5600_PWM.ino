#include <Wire.h>
#include <util/atomic.h>

const int PWM_PIN = 3;
const int AS5600_ADDR = 0x36;

volatile unsigned long riseTime = 0;
volatile unsigned long highTime = 0;
volatile unsigned long totalPeriod = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  //modifies the AS5600 configuration register for PWM mode
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x08); //the configuration register
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 1); //1 byte to read
  if (Wire.available() == 0) {
    Serial.println("ERROR: AS5600 not found on I2C bus!");
    while(1);
  }
  uint8_t currentSettings = Wire.read();

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x08);
  Wire.write((currentSettings & 0b00001111) | 0b11100000); //switch 4 and 5 configured to 10 for PWM output, 6 and 7 to 11 for 920 hz
  Wire.endTransmission();

  //setup the interrupt using a pwm interrupt pin
  pinMode(PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), catchWave, CHANGE);
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

void loop() {
  unsigned long myHigh, myPeriod;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    myHigh = highTime;
    myPeriod = totalPeriod;
  }  

  if (myPeriod == 0) return; //waits for the first rising edge to be detected

  long clockTicks = (myHigh * 4351) / myPeriod; //4351 ticks for full wave (divide high and period for the current amount of ticks) (or 4127)
  
  int rawAngle = clockTicks - 128; //0 degrees starts at 128 ticks (or 16)

  if (rawAngle < 0) rawAngle = 0;

  if (rawAngle > 4095) rawAngle = 4095;

  float degrees = rawAngle * (360.0 / 4096.0);

  Serial.print("Angle: ");
  Serial.println(degrees);
  delay(100);
}