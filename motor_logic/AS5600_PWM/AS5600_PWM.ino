#include <Wire.h>
#include <util/atomic.h>

//Hardware
const int PWM_PIN = 3;                  
const int MULTIPLEXER_ADDR = 0x70;  //check
const int AS5600_ADDR = 0x36;           
const int CONF_REG_LOW = 0x08;  //the configuration register

//Interrupt Variables
volatile unsigned long riseTime = 0;
volatile unsigned long highTime = 0;
volatile unsigned long period = 1086; //920Hz

//Timing
unsigned long prevPrintMillis = 0;

// MULTIPLEXER HELPER
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MULTIPLEXER_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// AS5600 CONFIGURATION
void setAS5600ToPWM() {
  tcaSelect(0); // Open channel 0 for Motor A

  //modifies the AS5600 configuration register for PWM mode
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(CONF_REG_LOW); 
  Wire.endTransmission(false);

  Wire.requestFrom(AS5600_ADDR, 1); //1 byte to read
  if (Wire.available() == 0) {
    Serial.println("ERROR: AS5600 not found on I2C bus!");
    while(1);
  }
  uint8_t confLow = Wire.read();
  
  confLow = (confLow & 0b00001111) | 0b11100000; //bits 5-4 -> 10 for PWM, bits 7-6 -> 11 for 920Hz

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(CONF_REG_LOW);
  Wire.write(confLow);
  Wire.endTransmission();
  
  Serial.println("SUCCESS: AS5600 configured to PWM Mode.");
}

// HARDWARE INTERRUPT ROUTINE
void readPWM() {
  unsigned long now = micros();
  if (digitalRead(PWM_PIN) == HIGH) {
    period = now - riseTime; // How long since the last HIGH
    riseTime = now; // Reset the timer
  } else {
    highTime = now - riseTime; // HIGH duration
  }
}

// MAIN SETUP
void setup() {
  Serial.begin(115200);
  Wire.begin();

  setAS5600ToPWM();

  //setup the interrupt using a pwm interrupt pin
  pinMode(PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), readPWM, CHANGE);
  
  Serial.println("Starting PWM telemetry read...");
}

// MAIN LOOP
void loop() {
  if (millis() - prevPrintMillis >= 200) { // print 5 times a second
    prevPrintMillis = millis();
    
    unsigned long safeHigh, safePeriod;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      safeHigh = highTime;
      safePeriod = period;
    }
    if (safePeriod == 0) return; //waits for the first rising edge to be detected

    long clockCount = (safeHigh * 4351) / safePeriod; //4351 ticks for full wave (divide high and period for the current amount of ticks)
    
    int rawAngle = clockCount - 128; //0 degrees starts at 128 ticks

    if (rawAngle < 0) rawAngle = 0;
    if (rawAngle > 4095) rawAngle = 4095;

    float degrees = rawAngle * (360.0 / 4096.0);

    Serial.print("High(us): ");
    Serial.print(safeHigh);
    Serial.print("\t Period(us): ");
    Serial.print(safePeriod);
    Serial.print("\t Raw(0-4095): ");
    Serial.print(rawAngle);
    Serial.print("\t Angle: ");
    Serial.println(degrees);
  }
}