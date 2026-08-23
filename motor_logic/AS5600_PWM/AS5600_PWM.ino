#include <Wire.h>
#include <util/atomic.h>

// --- Hardware Definitions ---
const int PWM_PIN = 3;                  // Mega Interrupt Pin 1
const int MULTIPLEXER_ADDR = 0x70;      // TCA9548A I2C address
const int AS5600_ADDR = 0x36;           // AS5600 fixed I2C address
const int CONF_REG_LOW = 0x08;          // Register holding the output mode bits

// --- Volatile Interrupt Variables ---
volatile unsigned long riseTime = 0;
volatile unsigned long highTime = 0;
volatile unsigned long period = 1086;   // Expected default period at 920Hz

// --- Timing for Serial Print ---
unsigned long prevPrintMillis = 0;

// ==========================================
// MULTIPLEXER HELPER
// ==========================================
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MULTIPLEXER_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ==========================================
// AS5600 CONFIGURATION (Runs once)
// ==========================================
void setAS5600ToPWM() {
  tcaSelect(0); // Open channel 0 for Motor A
  
  // 1. Read current CONF register
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(CONF_REG_LOW); 
  Wire.endTransmission(false);
  
  Wire.requestFrom(AS5600_ADDR, 1);
  if (Wire.available() == 0) {
    Serial.println("ERROR: AS5600 not found on I2C bus!");
    while(1); // Halt execution if sensor is unplugged
  }
  uint8_t confLow = Wire.read();
  
  // 2. Modify Output Mode bits (Bits 5:4 -> 01 for PWM)
  // Bitwise AND clears bits 5 and 4. Bitwise OR sets bit 4 to HIGH.
  confLow = (confLow & 0b11001111) | 0b00010000; 
  
  // 3. Write it back to the sensor
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(CONF_REG_LOW);
  Wire.write(confLow);
  Wire.endTransmission();
  
  Serial.println("SUCCESS: AS5600 configured to PWM Mode.");
}

// ==========================================
// HARDWARE INTERRUPT ROUTINE
// ==========================================
void readPWM() {
  unsigned long now = micros();
  if (digitalRead(PWM_PIN) == HIGH) {
    // The wave just went HIGH. The time since the LAST time it went HIGH is our total period.
    period = now - riseTime; 
    riseTime = now;
  } else {
    // The wave just went LOW. The time it spent HIGH is our pulse width.
    highTime = now - riseTime; 
  }
}

// ==========================================
// MAIN SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Configure the sensor
  setAS5600ToPWM();
  
  // Attach the interrupt to Pin 3
  pinMode(PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), readPWM, CHANGE);
  
  Serial.println("Starting PWM telemetry read...");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  // Only print 5 times a second to keep the Serial Monitor readable
  if (millis() - prevPrintMillis >= 200) {
    prevPrintMillis = millis();
    
    unsigned long safeHigh, safePeriod;
    
    // ATOMIC_BLOCK pauses the interrupt just long enough to copy the variables safely
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      safeHigh = highTime;
      safePeriod = period;
    }
    
    // Failsafe: Prevent divide-by-zero if the interrupt hasn't triggered yet
    if (safePeriod == 0) return;

    // --- The Datasheet Math ---
    // Total clocks in a period = 4127. 
    // We multiply before dividing to avoid floating point math slow-downs.
    long clockCount = (safeHigh * 4127) / safePeriod;
    
    // 0 degrees is exactly 16 clocks. 
    int rawAngle = clockCount - 16;
    
    // Clamp values in case of microscopic timing jitter
    if (rawAngle < 0) rawAngle = 0;
    if (rawAngle > 4095) rawAngle = 4095;
    
    // Convert to human-readable degrees for testing
    float degrees = rawAngle * (360.0 / 4096.0);
    
    // Print diagnostics
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