#include <util/atomic.h>
#include <math.h> 

// ==========================================
// --- MOTION PROFILING (STEPPER FEEL) ---
// ==========================================
// Controls how fast the setpoint moves. Lower = Slower & Smoother.
const float MAX_SPEED_A = 3.0;   // Pulses/sec (PPR 80) -> ~13.5 deg/sec
const float MAX_SPEED_B = 500.0; // Pulses/sec (PPR 17520) -> ~10.2 deg/sec
const float MAX_SPEED_C = 500.0; // Pulses/sec (PPR 17520) -> ~10.2 deg/sec
const float MAX_SPEED_D = 3.0;   // Pulses/sec (PPR 80) -> ~13.5 deg/sec

// ==========================================
// --- MOTOR ERROR COMPENSATION CONFIG ---
// ==========================================
const int OFFSET_DROP_BELOW = 0;      
const int OFFSET_DROP_ABOVE = 0;      
const int MIN_COMPENSATED_ANGLE = -15; //prone to change when using AS5600
const int MAX_COMPENSATED_ANGLE = 195; //prone to change when using AS5600

// ==========================================
// MOTOR A: BIGMOTOR (SHOULDER JOINT)
// ==========================================
const int clkPinA = 3;  
const int dtPinA = 21;
#define IN1_A 7
#define IN2_A 6
#define PPR_A 80.0 
#define TOLERANCE_A 1  

volatile long counterA = 0; 
float eprevA = 0.0;
float eintegralA = 0.0;
float filtered_dedtA = 0.0; 

float currentSetpointA = 0.0; 
float finalTargetA = 0.0;     
int targetAngleA = 90; //prone to change since IK does not start at 90 degrees
bool targetReachedA = true;

float kpA = 12.0; 
float kiA = 9.0;        
float kdA = 1.6; 
const float integralLimitA = 200.0;

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
const float integralLimitB = 1600.0;

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
int targetAngleC = 90; //prone to change since IK does not start at 90 degrees
bool targetReachedC = true;

float kpC = 0.6965; 
float kiC = 0.06;        
float kdC = 0.0070; 
const float integralLimitC = 1600.0;

// ==========================================
// MOTOR D: BIGMOTOR (ELBOW JOINT)
// ==========================================
const int clkPinD = 19;  
const int dtPinD = 20;
#define IN1_D 11
#define IN2_D 12
#define PPR_D 80.0 
#define TOLERANCE_D 1   

volatile long counterD = 0; 
float eprevD = 0.0;
float eintegralD = 0.0;
float filtered_dedtD = 0.0; 

float currentSetpointD = 0.0; 
float finalTargetD = 0.0;     
int targetAngleD = 90; //prone to change since IK does not start at 90 degrees
bool targetReachedD = true;

float kpD = 12.0; 
float kiD = 9.0;        
float kdD = 1.6; 
const float integralLimitD = 200.0;

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
void readEncoderA();
void readEncoderB();
void readEncoderC();
void readEncoderD();
void setMotorA(int dir, int pwmVal);
void setMotorB(int dir, int pwmVal);
void setMotorC(int dir, int pwmVal);
void setMotorD(int dir, int pwmVal);
void readSerialTarget();

void setup() {
  Serial.begin(115200);

  // Setup Motor A
  pinMode(clkPinA, INPUT_PULLUP);
  pinMode(dtPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(clkPinA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dtPinA), readEncoderA, CHANGE);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  
  counterA = (long)(90.0 * (PPR_A / 360.0)); //(prone to change since IK does not start at 90 degrees)
  currentSetpointA = counterA; //the smaller target positions to reach the final target position
  finalTargetA = counterA; //final target position
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
  
  counterC = (long)(90.0 * (PPR_C / 360.0)); //(prone to change since IK does not start at 90 degrees)
  currentSetpointC = counterC; 
  finalTargetC = counterC; 
  setMotorC(0, 0);

  // Setup Motor D
  pinMode(clkPinD, INPUT_PULLUP);
  pinMode(dtPinD, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(clkPinD), readEncoderD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dtPinD), readEncoderD, CHANGE);
  pinMode(IN1_D, OUTPUT);
  pinMode(IN2_D, OUTPUT);
  
  counterD = (long)(90.0 * (PPR_D / 360.0)); //(prone to change since IK does not start at 90 degrees)
  currentSetpointD = counterD; 
  finalTargetD = counterD; 
  setMotorD(0, 0);

  Serial.println("Quad Motor PID System Ready.");
  Serial.println("Commands: A<angle>, B<angle>, C<angle>, D<angle>. Ex: A45 B180 C0 D90");
  Serial.println("WARNING: Ensure Motor A, C, and D are pointing STRAIGHT UP (90 deg) before starting!");
  //when prone to change, the joints straight up will be 0 degrees (the range is -90 to 90)

  prevPidMicros = micros();
}

void loop() {
  digitalWrite(STDBY, HIGH); //for Motor C
  readSerialTarget();

  unsigned long now = micros();
  if (now - prevPidMicros < PID_SAMPLE_US) return; //returns to top loop 
  
  float deltaT = (now - prevPidMicros) / 1000000.0; //seconds
  prevPidMicros = now;

  long posA = 0, posB = 0, posC = 0, posD = 0; //the variable used to store counter variables safely
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posA = counterA; 
    posB = counterB;
    posC = counterC;
    posD = counterD;
  }

  // =========================================================
  // --- PID MOTOR A (BIGMOTOR - SHOULDER) ---
  // =========================================================
  if (!targetReachedA) {
    
    // SLEW RATE LIMITER WITH DECELERATION ZONE
    if (currentSetpointA != finalTargetA) {
      float distanceToTargetA = abs(finalTargetA - currentSetpointA);
      float decelerationZoneA = 4.0; // Optimized for 80 PPR (~18 degrees)
      float currentMaxSpeedA = MAX_SPEED_A;

      if (distanceToTargetA < decelerationZoneA) {
        currentMaxSpeedA = max(0.5f, MAX_SPEED_A * (distanceToTargetA / decelerationZoneA)); //range between 0.5 PPR to 3.0 (2.4 to 13.5 deg/sec)
      }

      float stepA = currentMaxSpeedA * deltaT; 

      if (distanceToTargetA <= stepA) {
        currentSetpointA = finalTargetA;  //(prone to change)
      } else if (finalTargetA > currentSetpointA) {
        currentSetpointA += stepA;  //updates the small target position by each allowable rotation/step (prone to change)
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
      float alpha = 0.15; //prone to change depending on the outcome of AS5600 
      filtered_dedtA = (alpha * raw_dedtA) + ((1.0 - alpha) * filtered_dedtA); //prone to change depending on the outcome of AS5600

      eintegralA += eA * deltaT;
      if (eintegralA > integralLimitA) eintegralA = integralLimitA; //(prone to change)
      if (eintegralA < -integralLimitA) eintegralA = -integralLimitA; //(prone to change)
      
      float uA_pid = kpA * eA + kdA * filtered_dedtA + kiA * eintegralA;

      // CONDITIONAL GRAVITY COMPENSATION
      float kG = 30.0;
      float uG = 0.0;
      float currentAngleDegA = posA * (360.0 / PPR_A);
      
      // True if moving up towards 90 degrees from either side
      bool isLiftingA = (currentAngleDegA < 90.0 && currentSetpointA > posA) || //(prone to change since IK does not start at 90 degrees)
                        (currentAngleDegA > 90.0 && currentSetpointA < posA);
      
      if (isLiftingA) { 
          uG = kG * cos(currentAngleRad); 
      }
      
      float uA_total = uA_pid + uG; //check validity since normal IK can range to -90 not 0 degrees

      if (uA_total > PWM_LIMIT) uA_total = PWM_LIMIT;
      if (uA_total < -PWM_LIMIT) uA_total = -PWM_LIMIT;

      int dirA = (uA_total > 0) ? 1 : ((uA_total < 0) ? -1 : 0); //check if necessary since the motors are self-locking
      setMotorA(dirA, abs((int)uA_total));
      eprevA = eA; 
    }
  }

  // =========================================================
  // --- PID MOTOR B (BASE JGA25) ---
  // =========================================================
  if (!targetReachedB) {
    
    // SLEW RATE LIMITER WITH DECELERATION ZONE
    if (currentSetpointB != finalTargetB) {
      float distanceToTargetB = abs(finalTargetB - currentSetpointB);
      float decelerationZoneB = 1000.0; // Appropriate for high PPR JGA25
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
      float alphaB = 0.15; //prone to change depending on the outcome of AS5600
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
    
    // SLEW RATE LIMITER WITH DECELERATION ZONE
    if (currentSetpointC != finalTargetC) {
      float distanceToTargetC = abs(finalTargetC - currentSetpointC);
      float decelerationZoneC = 1000.0; // Appropriate for high PPR JGA25
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
      
      // Removed gravity for distal wrist - PID + slew handles it cleanly
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
    
    // SLEW RATE LIMITER WITH DECELERATION ZONE
    if (currentSetpointD != finalTargetD) {
      float distanceToTargetD = abs(finalTargetD - currentSetpointD);
      float decelerationZoneD = 4.0; // Optimized for 80 PPR (~18 degrees)
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

      // RESTORED CONDITIONAL GRAVITY COMPENSATION
      float kG_D = 30.0; 
      float uG_D = 0.0;
      float currentAngleDegD = posD * (360.0 / PPR_D);
      
      // True if moving up towards 90 degrees from either side
      bool isLiftingD = (currentAngleDegD < 90.0 && currentSetpointD > posD) || 
                        (currentAngleDegD > 90.0 && currentSetpointD < posD);
      
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
// ENCODER ISR ROUTINES
// ==========================================
void readEncoderA() { //prone to change for AS5600
  static uint8_t old_state = 0;
  uint8_t current_A = digitalRead(clkPinA);
  uint8_t current_B = digitalRead(dtPinA);
  uint8_t current_state = (current_A << 1) | current_B;
  uint8_t movement = (old_state << 2) | current_state;
  if (movement == 0b0001 || movement == 0b0111 || movement == 0b1110 || movement == 0b1000) counterA++;
  else if (movement == 0b0010 || movement == 0b1011 || movement == 0b1101 || movement == 0b0100) counterA--;
  old_state = current_state;
}

void readEncoderB() {
  if (digitalRead(ENCB_B) > 0) counterB++; else counterB--;
}

void readEncoderC() {
  if (digitalRead(ENCB_C) > 0) counterC++; else counterC--;
}

void readEncoderD() { //prone to change for AS5600
  static uint8_t old_state = 0;
  uint8_t current_A = digitalRead(clkPinD);
  uint8_t current_B = digitalRead(dtPinD);
  uint8_t current_state = (current_A << 1) | current_B;
  uint8_t movement = (old_state << 2) | current_state;
  if (movement == 0b0001 || movement == 0b0111 || movement == 0b1110 || movement == 0b1000) counterD++;
  else if (movement == 0b0010 || movement == 0b1011 || movement == 0b1101 || movement == 0b0100) counterD--;
  old_state = current_state;
}

void readSerialTarget() { //DONE
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
          if (rawInputA < 0) rawInputA = 0;
          if (rawInputA > 180) rawInputA = 180;
          
          static int lastRawInputA = 90; //starts at 90 degrees (prone to change, since IK does not start at 90 degrees)
          static int currentOffsetA = 0; 
          
          if (lastRawInputA == 90 && rawInputA != 90) { //add the target by a value offset when starting at 90 degrees (prone to change when using AS5600)
              if (rawInputA < 90) currentOffsetA = OFFSET_DROP_BELOW;
              else currentOffsetA = OFFSET_DROP_ABOVE;
          } else if (rawInputA != lastRawInputA) { //anything else besides 90 will not have offset (prone to change when using AS5600)
              currentOffsetA = 0;
          }
          lastRawInputA = rawInputA;

          int compensatedTarget = rawInputA + currentOffsetA;
          if (compensatedTarget < MIN_COMPENSATED_ANGLE) compensatedTarget = MIN_COMPENSATED_ANGLE;
          if (compensatedTarget > MAX_COMPENSATED_ANGLE) compensatedTarget = MAX_COMPENSATED_ANGLE;
          
          targetAngleA = compensatedTarget;
          finalTargetA = targetAngleA * (PPR_A / 360.0); //will be used in the main loop for target position
          targetReachedA = false;
        }
        
        // --- Parse JGA25 (B) ---
        if (bIndex != -1) {
          targetAngleB = inputString.substring(bIndex + 1).toInt();
          finalTargetB = targetAngleB * (PPR_B / 360.0); //will be used in the main loop for target position
          targetReachedB = false;
        }

        // --- Parse JGA25 (C) ---
        if (cIndex != -1) {
          int rawInputC = inputString.substring(cIndex + 1).toInt();
          if (rawInputC < 0) rawInputC = 0;
          if (rawInputC > 180) rawInputC = 180;
          
          targetAngleC = rawInputC;
          finalTargetC = targetAngleC * (PPR_C / 360.0); //will be used in the main loop for target position
          targetReachedC = false;
        }
        
        // --- Parse BIGMOTOR (D) ---
        if (dIndex != -1) {
          int rawInputD = inputString.substring(dIndex + 1).toInt();
          if (rawInputD < 0) rawInputD = 0;
          if (rawInputD > 180) rawInputD = 180;
          
          static int lastRawInputD = 90; 
          static int currentOffsetD = 0; 
          
          if (lastRawInputD == 90 && rawInputD != 90) {
              if (rawInputD < 90) currentOffsetD = OFFSET_DROP_BELOW; //same as Bigmotor A's situation (prone to change when using AS5600)
              else currentOffsetD = OFFSET_DROP_ABOVE;
          } else if (rawInputD != lastRawInputD) {
              currentOffsetD = 0;
          }
          lastRawInputD = rawInputD;

          int compensatedTargetD = rawInputD + currentOffsetD;
          if (compensatedTargetD < MIN_COMPENSATED_ANGLE) compensatedTargetD = MIN_COMPENSATED_ANGLE;
          if (compensatedTargetD > MAX_COMPENSATED_ANGLE) compensatedTargetD = MAX_COMPENSATED_ANGLE;
          
          targetAngleD = compensatedTargetD;
          finalTargetD = targetAngleD * (PPR_D / 360.0); //will be used in the main loop for target position
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
