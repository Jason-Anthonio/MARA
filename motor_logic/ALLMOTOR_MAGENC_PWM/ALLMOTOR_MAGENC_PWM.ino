// Motors A (shoulder) and D (elbow) now read ABSOLUTE position from an
// AS5600 magnetic encoder via its PWM output
//
//   2. CHECK DIRECTION.
//      AS5600_INVERT_A / _D default to false. If a joint moves the wrong
//      way relative to the angle you commanded, flip the matching one to
//      true and re-test.
//
//   3. I2C MUX FOR THE TWO AS5600 CHIPS.
//      Both AS5600 chips share the same fixed I2C address (0x36), so a
//      TCA9548A I2C multiplexer (a chip that lets one bus reach several
//      downstream devices, each on its own selectable channel) sits
//      between the Mega and both sensors. configureAS5600ForPWM() selects
//      the right mux channel before each chip's one-time config write.
//      Wiring assumed below: mux upstream SDA/SCL -> Mega SDA/SCL, Motor
//      A's AS5600 on mux channel 0, Motor D's AS5600 on mux channel 1, mux
//      address 0x70 (default with A0-A2 tied low). Adjust the #defines if
//      your wiring differs. The mux is only used during setup() -- once
//      each chip is in PWM mode, position reads happen over plain GPIO
//      (AS5600_PWM_PIN_A/D), no further I2C/mux traffic needed.

#include <Wire.h>
#include <util/atomic.h>
#include <math.h>

// ==========================================
// --- MOTION PROFILING (STEPPER FEEL) ---
// ==========================================
// Controls how fast the setpoint moves. Lower = Slower & Smoother.
const float MAX_SPEED_A = 114.0;   // Pulses/sec (PPR 4096) -> ~10.01 deg/sec
const float MAX_SPEED_B = 500.0; // Pulses/sec (PPR 17520) -> ~10.2 deg/sec
const float MAX_SPEED_C = 500.0; // Pulses/sec (PPR 17520) -> ~10.2 deg/sec
const float MAX_SPEED_D = 114.0;   // Pulses/sec (PPR 4096) -> ~10.01 deg/sec

// ==========================================
// --- MOTOR ERROR COMPENSATION CONFIG ---
// ==========================================
const int OFFSET_DROP_BELOW = 0;      
const int OFFSET_DROP_ABOVE = 0;      
const int MIN_COMPENSATED_ANGLE = 0;
const int MAX_COMPENSATED_ANGLE = 180;

// ==========================================
// --- AS5600 SHARED CONFIG (MOTOR A & D) ---
// ==========================================
#define AS5600_ADDR 0x36   // fixed I2C address, same for every stock AS5600

// TCA9548A I2C mux -- lets the shared-address AS5600 chips be configured
// individually. Only used during setup(); position reads afterward are
// plain GPIO PWM decode, no mux involved.
#define TCA9548A_ADDR 0x70        // default mux address (A0-A2 tied low)
#define AS5600_MUX_CHANNEL_A 0    // mux channel Motor A's AS5600 is wired to
#define AS5600_MUX_CHANNEL_D 1    // mux channel Motor D's AS5600 is wired to

// ==========================================
// MOTOR A: BIGMOTOR (SHOULDER JOINT) — AS5600 absolute magnetic encoder
// ==========================================
#define IN1_A 7
#define IN2_A 6
#define PPR_A 4096.0   // AS5600 native resolution: 4096 ticks per revolution
#define TOLERANCE_A 6 

// AS5600 PWM decode (replaces the old two-pin quadrature readEncoderA)
#define AS5600_PWM_PIN_A 3   // interrupt-capable pin (Mega: 2,3,18,19,20,21)
volatile unsigned long riseTimeA = 0;
volatile unsigned long highTimeA = 0;
volatile unsigned long totalPeriodA = 0;
long lastGoodPosA = 0;  // holds the last valid reading if a PWM cycle is missed

// Calibration -- see notes at top of file.
int AS5600_ZERO_OFFSET_A = 128;   // 0 deg per datasheet; not being recalibrated
bool AS5600_INVERT_A = false;     // verify direction on the bench

float eprevA = 0.0;
float eintegralA = 0.0;
float filtered_dedtA = 0.0; 

float currentSetpointA = 0.0; 
float finalTargetA = 0.0;     
int targetAngleA = 0;
bool targetReachedA = true;

float kpA = 0.23;
float kiA = 0.17;        
float kdA = 0.03;
const float integralLimitA = 332.0;

// ==========================================
// MOTOR B: JGA25 (BASE JOINT)
// ==========================================
#define ENCA_B 2
#define ENCB_B 22
#define PWM_PIN_B 8
#define IN1_B 23
#define IN2_B 24
#define PPR_B 17520.0 
#define TOLERANCE_B 16 

volatile long counterB = 0;
float eprevB = 0.0;
float eintegralB = 0.0;
float filtered_dedtB = 0.0;

float currentSetpointB = 0.0; 
float finalTargetB = 0.0;     
int targetAngleB = 0; 
bool targetReachedB = true;

float kpB = 0.6965;
float kiB = 0.06;        
float kdB = 0.0070;
const float integralLimitB = 110.0;

// ==========================================
// MOTOR C: JGA25 (WRIST JOINT)
// ==========================================
#define ENCA_C 18 
#define ENCB_C 25
#define PWM_PIN_C 5 
#define IN1_C 26 
#define IN2_C 27
#define PPR_C 17520.0 
#define STDBY 28
#define TOLERANCE_C 16 

volatile long counterC = 0; 
float eprevC = 0.0;
float eintegralC = 0.0;
float filtered_dedtC = 0.0; 

float currentSetpointC = 0.0; 
float finalTargetC = 0.0;     
int targetAngleC = 0;
bool targetReachedC = true;

float kpC = 0.6965; 
float kiC = 0.06;        
float kdC = 0.0070; 
const float integralLimitC = 110.0;

// ==========================================
// MOTOR D: BIGMOTOR (ELBOW JOINT) — AS5600 absolute magnetic encoder
// ==========================================
#define IN1_D 11
#define IN2_D 12
#define PPR_D 4096.0 
#define TOLERANCE_D 6   

#define AS5600_PWM_PIN_D 19  // interrupt-capable pin (Mega: 2,3,18,19,20,21)
volatile unsigned long riseTimeD = 0;
volatile unsigned long highTimeD = 0;
volatile unsigned long totalPeriodD = 0;
long lastGoodPosD = 0;

int AS5600_ZERO_OFFSET_D = 128;   // 0 deg per datasheet; not being recalibrated
bool AS5600_INVERT_D = false;     // verify direction on the bench

float eprevD = 0.0;
float eintegralD = 0.0;
float filtered_dedtD = 0.0; 

float currentSetpointD = 0.0; 
float finalTargetD = 0.0;     
int targetAngleD = 0;
bool targetReachedD = true;

float kpD = 0.23; 
float kiD = 0.17;        
float kdD = 0.03;
const float integralLimitD = 332.0;

// ==========================================
// GLOBAL SYSTEM VARIABLES
// ==========================================
const unsigned long PID_SAMPLE_US = 10000;
const unsigned long PRINT_MS = 100;
const int PWM_LIMIT = 255;

unsigned long prevPidMicros = 0;
unsigned long prevPrintMillis = 0;
String inputString = "";

// Function Prototypes
void selectMuxChannel(uint8_t channel);
void configureAS5600ForPWM(uint8_t muxChannel);
void catchWaveA();
void catchWaveD();
long readAS5600TicksA();
long readAS5600TicksD();
void readEncoderB();
void readEncoderC();
void setMotorA(int dir, int pwmVal);
void setMotorB(int dir, int pwmVal);
void setMotorC(int dir, int pwmVal);
void setMotorD(int dir, int pwmVal);
void readSerialTarget();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  configureAS5600ForPWM(AS5600_MUX_CHANNEL_A);   // one-time PWM-mode config, Motor A's chip
  configureAS5600ForPWM(AS5600_MUX_CHANNEL_D);   // one-time PWM-mode config, Motor D's chip

  // Setup Motor A (AS5600 absolute encoder)
  pinMode(AS5600_PWM_PIN_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(AS5600_PWM_PIN_A), catchWaveA, CHANGE);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  delay(5); // give the sensor a moment to produce its first PWM cycle
  {
    long initPosA = readAS5600TicksA();
    if (initPosA < 0) initPosA = 0; // no signal yet, will self-correct next PID tick
    lastGoodPosA = initPosA;
    currentSetpointA = initPosA;    // start the setpoint at the arm's real position
    finalTargetA = initPosA;
  }
  setMotorA(0, 0);

  // Setup Motor B
  pinMode(ENCA_B, INPUT);
  pinMode(ENCB_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), readEncoderB, RISING);
  pinMode(PWM_PIN_B, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);
  
  counterB = 0; //counter is the real encoder values read constantly
  currentSetpointB = counterB; 
  finalTargetB = counterB; 
  setMotorB(0, 0);

  // Setup Motor C
  pinMode(STDBY, OUTPUT);
  pinMode(ENCA_C, INPUT);
  pinMode(ENCB_C, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_C), readEncoderC, RISING);
  pinMode(PWM_PIN_C, OUTPUT);
  pinMode(IN1_C, OUTPUT);
  pinMode(IN2_C, OUTPUT);
  
  counterC = (long)(0.0 * (PPR_C / 360.0));
  currentSetpointC = counterC; 
  finalTargetC = counterC; 
  setMotorC(0, 0);

  // Setup Motor D (AS5600 absolute encoder)
  pinMode(AS5600_PWM_PIN_D, INPUT);
  attachInterrupt(digitalPinToInterrupt(AS5600_PWM_PIN_D), catchWaveD, CHANGE);
  pinMode(IN1_D, OUTPUT);
  pinMode(IN2_D, OUTPUT);
  delay(5);
  {
    long initPosD = readAS5600TicksD();
    if (initPosD < 0) initPosD = 0;
    lastGoodPosD = initPosD;
    currentSetpointD = initPosD;
    finalTargetD = initPosD;
  }
  setMotorD(0, 0);

  Serial.println("Quad Motor PID System Ready.");
  Serial.println("Commands: A<angle>, B<angle>, C<angle>, D<angle>. Ex: A45 B180 C0 D90");
  Serial.println("WARNING: Ensure Motor C is pointing STRAIGHT UP (0 deg) before starting!");
  Serial.println("NOTE: Motors A and D read their real position from the AS5600 at boot -- no manual zeroing needed once AS5600_ZERO_OFFSET_A/D are calibrated.");

  prevPidMicros = micros();
}

void loop() {
  digitalWrite(STDBY, HIGH); //for Motor C
  readSerialTarget();

  unsigned long now = micros();
  if (now - prevPidMicros < PID_SAMPLE_US) return; //returns to top loop 
  
  float deltaT = (now - prevPidMicros) / 1000000.0; //seconds
  prevPidMicros = now;

  long posA, posB = 0, posC = 0, posD;

  // Motor A/D: absolute AS5600 reading. If a fresh PWM cycle hasn't landed
  // yet this tick, reuse the last good value instead of snapping to 0.
  long rawA = readAS5600TicksA();
  if (rawA >= 0) lastGoodPosA = rawA;
  posA = lastGoodPosA;

  long rawD = readAS5600TicksD();
  if (rawD >= 0) lastGoodPosD = rawD;
  posD = lastGoodPosD;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posB = counterB;
    posC = counterC;
  }

  // =========================================================
  // --- PID MOTOR A (BIGMOTOR - SHOULDER) ---
  // =========================================================
  if (!targetReachedA) {
    
    // SLEW RATE LIMITER WITH DECELERATION ZONE
    if (currentSetpointA != finalTargetA) {
      float distanceToTargetA = abs(finalTargetA - currentSetpointA);
      float decelerationZoneA = 171.0; // Optimized for 4096 PPR (~15 degrees)
      float currentMaxSpeedA = MAX_SPEED_A;

      if (distanceToTargetA < decelerationZoneA) {
        currentMaxSpeedA = max(0.5f, MAX_SPEED_A * (distanceToTargetA / decelerationZoneA));
      }

      float stepA = currentMaxSpeedA * deltaT; 

      if (distanceToTargetA <= stepA) {
        currentSetpointA = finalTargetA;
      } else if (finalTargetA > currentSetpointA) {
        currentSetpointA += stepA;
      } else {
        currentSetpointA -= stepA; 
      }
    }

    float eA = currentSetpointA - posA; 
    
    if (currentSetpointA == finalTargetA && fabs(eA) <= TOLERANCE_A) { 
      setMotorA(0, 0);
      targetReachedA = true;
      eintegralA = 0;
      eprevA = 0;
      filtered_dedtA = 0; 
    } else {
      float currentAngleRad = (posA * (360.0 / PPR_A)) * (PI / 180.0);
      
      float raw_dedtA = (eA - eprevA) / deltaT;
      float alpha = 0.15;
      filtered_dedtA = (alpha * raw_dedtA) + ((1.0 - alpha) * filtered_dedtA);

      eintegralA += eA * deltaT;
      if (eintegralA > integralLimitA) eintegralA = integralLimitA;
      if (eintegralA < -integralLimitA) eintegralA = -integralLimitA;
      
      float uA_pid = kpA * eA + kdA * filtered_dedtA + kiA * eintegralA;

      // CONDITIONAL GRAVITY COMPENSATION
      float kG = 30.0;
      float uG = 0.0;
      float currentAngleDegA = posA * (360.0 / PPR_A);
      
      bool isLiftingA = (currentAngleDegA < 0.0 && currentSetpointA > posA) ||
                        (currentAngleDegA > 0.0 && currentSetpointA < posA);
      
      if (isLiftingA) { 
          uG = kG * cos(currentAngleRad); 
      }
      
      float uA_total = uA_pid + uG;

      if (uA_total > PWM_LIMIT) uA_total = PWM_LIMIT;
      if (uA_total < -PWM_LIMIT) uA_total = -PWM_LIMIT;

      int dirA = (uA_total > 0) ? 1 : ((uA_total < 0) ? -1 : 0);
      setMotorA(dirA, abs((int)uA_total));
      eprevA = eA; 
    }
  }

  // =========================================================
  // --- PID MOTOR B (BASE JGA25) ---
  // =========================================================
  if (!targetReachedB) {
    
    if (currentSetpointB != finalTargetB) {
      float distanceToTargetB = abs(finalTargetB - currentSetpointB);
      float decelerationZoneB = 1000.0;
      float currentMaxSpeedB = MAX_SPEED_B;

      if (distanceToTargetB < decelerationZoneB) {
        currentMaxSpeedB = max(20.0f, MAX_SPEED_B * (distanceToTargetB / decelerationZoneB)); 
      }

      float stepB = currentMaxSpeedB * deltaT; 

      if (distanceToTargetB <= stepB) {
        currentSetpointB = finalTargetB; 
      } else if (finalTargetB > currentSetpointB) {
        currentSetpointB += stepB; 
      } else {
        currentSetpointB -= stepB; 
      }
    }

    float eB = currentSetpointB - posB;
    
    if (currentSetpointB == finalTargetB && fabs(eB) <= TOLERANCE_B) {
      setMotorB(0, 0);
      targetReachedB = true;
      eintegralB = 0;
      eprevB = 0;
      filtered_dedtB = 0;
    } else {
      float raw_dedtB = (eB - eprevB) / deltaT;
      float alphaB = 0.15; 
      filtered_dedtB = (alphaB * raw_dedtB) + ((1.0 - alphaB) * filtered_dedtB);
      
      eintegralB += eB * deltaT;
      if (eintegralB > integralLimitB) eintegralB = integralLimitB;
      if (eintegralB < -integralLimitB) eintegralB = -integralLimitB;
      
      float uB = kpB * eB + kdB * filtered_dedtB + kiB * eintegralB;
      
      if (uB > PWM_LIMIT) uB = PWM_LIMIT;
      if (uB < -PWM_LIMIT) uB = -PWM_LIMIT;
      
      int dirB = (uB > 0) ? 1 : ((uB < 0) ? -1 : 0);
      setMotorB(dirB, abs((int)uB));
      eprevB = eB; 
    }
  }

  // =========================================================
  // --- PID MOTOR C (WRIST JGA25) ---
  // =========================================================
  if (!targetReachedC) {
    
    if (currentSetpointC != finalTargetC) {
      float distanceToTargetC = abs(finalTargetC - currentSetpointC);
      float decelerationZoneC = 1000.0;
      float currentMaxSpeedC = MAX_SPEED_C;

      if (distanceToTargetC < decelerationZoneC) {
        currentMaxSpeedC = max(20.0f, MAX_SPEED_C * (distanceToTargetC / decelerationZoneC));
      }

      float stepC = currentMaxSpeedC * deltaT; 

      if (distanceToTargetC <= stepC) {
        currentSetpointC = finalTargetC; 
      } else if (finalTargetC > currentSetpointC) {
        currentSetpointC += stepC; 
      } else {
        currentSetpointC -= stepC; 
      }
    }

    float eC = currentSetpointC - posC; 
    
    if (currentSetpointC == finalTargetC && fabs(eC) <= TOLERANCE_C) { 
      setMotorC(0, 0);
      targetReachedC = true;
      eintegralC = 0;
      eprevC = 0;
      filtered_dedtC = 0; 
    } else {
      float raw_dedtC = (eC - eprevC) / deltaT;
      float alphaC = 0.15; 
      filtered_dedtC = (alphaC * raw_dedtC) + ((1.0 - alphaC) * filtered_dedtC);

      eintegralC += eC * deltaT;
      if (eintegralC > integralLimitC) eintegralC = integralLimitC;
      if (eintegralC < -integralLimitC) eintegralC = -integralLimitC;
      
      float uC_total = kpC * eC + kdC * filtered_dedtC + kiC * eintegralC;

      if (uC_total > PWM_LIMIT) uC_total = PWM_LIMIT;
      if (uC_total < -PWM_LIMIT) uC_total = -PWM_LIMIT;

      int dirC = (uC_total > 0) ? 1 : ((uC_total < 0) ? -1 : 0);
      setMotorC(dirC, abs((int)uC_total));
      eprevC = eC; 
    }
  }

  // =========================================================
  // --- PID MOTOR D (BIGMOTOR ELBOW) ---
  // =========================================================
  if (!targetReachedD) {
    
    if (currentSetpointD != finalTargetD) {
      float distanceToTargetD = abs(finalTargetD - currentSetpointD);
      float decelerationZoneD = 171.0;
      float currentMaxSpeedD = MAX_SPEED_D;

      if (distanceToTargetD < decelerationZoneD) {
        currentMaxSpeedD = max(0.5f, MAX_SPEED_D * (distanceToTargetD / decelerationZoneD));
      }

      float stepD = currentMaxSpeedD * deltaT; 

      if (distanceToTargetD <= stepD) {
        currentSetpointD = finalTargetD; 
      } else if (finalTargetD > currentSetpointD) {
        currentSetpointD += stepD; 
      } else {
        currentSetpointD -= stepD; 
      }
    }

    float eD = currentSetpointD - posD; 
    
    if (currentSetpointD == finalTargetD && fabs(eD) <= TOLERANCE_D) { 
      setMotorD(0, 0);
      targetReachedD = true;
      eintegralD = 0;
      eprevD = 0;
      filtered_dedtD = 0; 
    } else {
      float currentAngleRadD = (posD * (360.0 / PPR_D)) * (PI / 180.0);

      float raw_dedtD = (eD - eprevD) / deltaT;
      float alphaD = 0.15;
      filtered_dedtD = (alphaD * raw_dedtD) + ((1.0 - alphaD) * filtered_dedtD);

      eintegralD += eD * deltaT;
      if (eintegralD > integralLimitD) eintegralD = integralLimitD;
      if (eintegralD < -integralLimitD) eintegralD = -integralLimitD;
      
      float uD_pid = kpD * eD + kdD * filtered_dedtD + kiD * eintegralD;

      float kG_D = 30.0;
      float uG_D = 0.0;
      float currentAngleDegD = posD * (360.0 / PPR_D);
      
      bool isLiftingD = (currentAngleDegD < 0.0 && currentSetpointD > posD) || 
                        (currentAngleDegD > 0.0 && currentSetpointD < posD);
      
      if (isLiftingD) { 
          uG_D = kG_D * cos(currentAngleRadD); 
      }
      
      float uD_total = uD_pid + uG_D;

      if (uD_total > PWM_LIMIT) uD_total = PWM_LIMIT;
      if (uD_total < -PWM_LIMIT) uD_total = -PWM_LIMIT;

      int dirD = (uD_total > 0) ? 1 : ((uD_total < 0) ? -1 : 0);
      setMotorD(dirD, abs((int)uD_total));
      eprevD = eD; 
    }
  }

  // --- DEBUG OUTPUT ---
  if (millis() - prevPrintMillis >= PRINT_MS) {
    if (!targetReachedA || !targetReachedB || !targetReachedC || !targetReachedD) {
      Serial.print("A[P:"); Serial.print(posA); 
      Serial.print(" T:"); Serial.print(finalTargetA); Serial.print("] | ");
      Serial.print("B[P:"); Serial.print(posB); 
      Serial.print(" T:"); Serial.print(finalTargetB); Serial.print("] | ");
      Serial.print("C[P:"); Serial.print(posC); 
      Serial.print(" T:"); Serial.print(finalTargetC); Serial.print("] | ");
      Serial.print("D[P:"); Serial.print(posD); 
      Serial.print(" T:"); Serial.print(finalTargetD); Serial.println("]");
    }
    prevPrintMillis = millis();
  }
}

// ==========================================
// HARDWARE CONTROL FUNCTIONS
// ==========================================
void setMotorA(int dir, int pwmVal) {
  if (dir == 1) { analogWrite(IN1_A, pwmVal); digitalWrite(IN2_A, LOW); } 
  else if (dir == -1) { digitalWrite(IN1_A, LOW); analogWrite(IN2_A, pwmVal); } 
  else { digitalWrite(IN1_A, LOW); digitalWrite(IN2_A, LOW); }
}

void setMotorB(int dir, int pwmVal) {
  analogWrite(PWM_PIN_B, pwmVal);
  if (dir == 1) { digitalWrite(IN1_B, HIGH); digitalWrite(IN2_B, LOW); } 
  else if (dir == -1) { digitalWrite(IN1_B, LOW); digitalWrite(IN2_B, HIGH); } 
  else { digitalWrite(IN1_B, LOW); digitalWrite(IN2_B, LOW); }
}

void setMotorC(int dir, int pwmVal) {
  analogWrite(PWM_PIN_C, pwmVal);
  if (dir == 1) { digitalWrite(IN1_C, HIGH); digitalWrite(IN2_C, LOW); } 
  else if (dir == -1) { digitalWrite(IN1_C, LOW); digitalWrite(IN2_C, HIGH); } 
  else { digitalWrite(IN1_C, LOW); digitalWrite(IN2_C, LOW); }
}

void setMotorD(int dir, int pwmVal) {
  if (dir == 1) { analogWrite(IN1_D, pwmVal); digitalWrite(IN2_D, LOW); } 
  else if (dir == -1) { digitalWrite(IN1_D, LOW); analogWrite(IN2_D, pwmVal); } 
  else { digitalWrite(IN1_D, LOW); digitalWrite(IN2_D, LOW); }
}

// ==========================================
// AS5600 PWM DECODE (MOTOR A & D)
// ==========================================

// Selects one downstream channel on the TCA9548A mux (0-7). Only that
// channel's device(s) are reachable on the I2C bus until the next select.
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// One-time config write: selects the given mux channel, then puts that
// channel's AS5600 CONF register into PWM output mode at ~920 Hz. The CONF
// register is volatile on the chip (resets on power loss), so this runs
// once per chip every boot.
void configureAS5600ForPWM(uint8_t muxChannel) {
  selectMuxChannel(muxChannel);

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x08); // the configuration register
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 1);
  if (Wire.available() == 0) {
    Serial.print("ERROR: AS5600 not found on mux channel ");
    Serial.println(muxChannel);
    return;
  }
  uint8_t currentSettings = Wire.read();

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x08);
  Wire.write((currentSettings & 0b00001111) | 0b11100000); // PWM out, ~920 Hz
  Wire.endTransmission();
}

// Interrupt service routines: on every edge of the PWM signal, timestamp it
// to work out how long the pulse was HIGH and how long the full period was.
void catchWaveA() {
  unsigned long rightNow = micros();
  if (digitalRead(AS5600_PWM_PIN_A) == HIGH) {
    totalPeriodA = rightNow - riseTimeA;
    riseTimeA = rightNow;
  } else {
    highTimeA = rightNow - riseTimeA;
  }
}

void catchWaveD() {
  unsigned long rightNow = micros();
  if (digitalRead(AS5600_PWM_PIN_D) == HIGH) {
    totalPeriodD = rightNow - riseTimeD;
    riseTimeD = rightNow;
  } else {
    highTimeD = rightNow - riseTimeD;
  }
}

// Applies the per-unit zero offset and direction flip, then clamps to the
// valid 0-4095 tick range.
long applyAS5600Calibration(long clockTicks, int zeroOffset, bool invert) {
  long rawAngle = clockTicks - zeroOffset;
  if (invert) rawAngle = 4096 - rawAngle;
  if (rawAngle < 0) rawAngle = 0;
  if (rawAngle > 4095) rawAngle = 4095;
  return rawAngle;
}

// Returns the current absolute position in ticks (0-4095), or -1 if no full
// PWM cycle has been captured yet (e.g. right after boot, or the sensor is
// disconnected). Callers should hold onto the last good value rather than
// treating -1 as a real position.
long readAS5600TicksA() {
  unsigned long myHigh, myPeriod;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    myHigh = highTimeA;
    myPeriod = totalPeriodA;
  }
  if (myPeriod == 0) return -1;
  long clockTicks = (myHigh * 4351) / myPeriod; // verify this constant against your PWM frequency setting
  return applyAS5600Calibration(clockTicks, AS5600_ZERO_OFFSET_A, AS5600_INVERT_A);
}

long readAS5600TicksD() {
  unsigned long myHigh, myPeriod;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    myHigh = highTimeD;
    myPeriod = totalPeriodD;
  }
  if (myPeriod == 0) return -1;
  long clockTicks = (myHigh * 4351) / myPeriod;
  return applyAS5600Calibration(clockTicks, AS5600_ZERO_OFFSET_D, AS5600_INVERT_D);
}

// ==========================================
// ENCODER ISR ROUTINES (MOTOR B & C, unchanged)
// ==========================================
void readEncoderB() {
  if (digitalRead(ENCB_B) > 0) counterB++; else counterB--;
}

void readEncoderC() {
  if (digitalRead(ENCB_C) > 0) counterC++; else counterC--;
}

void readSerialTarget() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      if (inputString.length() > 0) {
        
        inputString.toUpperCase(); 
        
        int aIndex = inputString.indexOf('A');
        int bIndex = inputString.indexOf('B');
        int cIndex = inputString.indexOf('C');
        int dIndex = inputString.indexOf('D');
        
        if (aIndex == -1 && bIndex == -1 && cIndex == -1 && dIndex == -1) {
          Serial.println("Error: Command must contain A, B, C, or D");
        }
        
        // --- Parse BIGMOTOR (A) ---
        if (aIndex != -1) {
          int rawInputA = inputString.substring(aIndex + 1).toInt();
          if (rawInputA < -90) rawInputA = -90;
          if (rawInputA > 90) rawInputA = 90;
          
          static int lastRawInputA = 0;
          static int currentOffsetA = 0; 
          
          if (lastRawInputA == 0 && rawInputA != 0) {
              if (rawInputA < 0) currentOffsetA = OFFSET_DROP_BELOW;
              else currentOffsetA = OFFSET_DROP_ABOVE;
          } else if (rawInputA != lastRawInputA) {
              currentOffsetA = 0;
          }
          lastRawInputA = rawInputA;

          int compensatedTarget = rawInputA + currentOffsetA;
          if (compensatedTarget < MIN_COMPENSATED_ANGLE) compensatedTarget = MIN_COMPENSATED_ANGLE;
          if (compensatedTarget > MAX_COMPENSATED_ANGLE) compensatedTarget = MAX_COMPENSATED_ANGLE;
          
          targetAngleA = compensatedTarget;
          finalTargetA = targetAngleA * (PPR_A / 360.0);
          targetReachedA = false;
        }
        
        // --- Parse JGA25 (B) ---
        if (bIndex != -1) {
          targetAngleB = inputString.substring(bIndex + 1).toInt();
          finalTargetB = targetAngleB * (PPR_B / 360.0);
          targetReachedB = false;
        }

        // --- Parse JGA25 (C) ---
        if (cIndex != -1) {
          int rawInputC = inputString.substring(cIndex + 1).toInt();
          if (rawInputC < -90) rawInputC = -90;
          if (rawInputC > 90) rawInputC = 90;
          
          targetAngleC = rawInputC;
          finalTargetC = targetAngleC * (PPR_C / 360.0);
          targetReachedC = false;
        }
        
        // --- Parse BIGMOTOR (D) ---
        if (dIndex != -1) {
          int rawInputD = inputString.substring(dIndex + 1).toInt();
          if (rawInputD < -90) rawInputD = -90;
          if (rawInputD > 90) rawInputD = 90;
          
          static int lastRawInputD = 0; 
          static int currentOffsetD = 0; 
          
          if (lastRawInputD == 0 && rawInputD != 0) {
              if (rawInputD < 0) currentOffsetD = OFFSET_DROP_BELOW;
              else currentOffsetD = OFFSET_DROP_ABOVE;
          } else if (rawInputD != lastRawInputD) {
              currentOffsetD = 0;
          }
          lastRawInputD = rawInputD;

          int compensatedTargetD = rawInputD + currentOffsetD;
          if (compensatedTargetD < MIN_COMPENSATED_ANGLE) compensatedTargetD = MIN_COMPENSATED_ANGLE;
          if (compensatedTargetD > MAX_COMPENSATED_ANGLE) compensatedTargetD = MAX_COMPENSATED_ANGLE;
          
          targetAngleD = compensatedTargetD;
          finalTargetD = targetAngleD * (PPR_D / 360.0);
          targetReachedD = false;
        }
      }
      inputString = ""; 
    } 
    else if (inChar != '\r') { 
      inputString += inChar;
    }
  }
}
