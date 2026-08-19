/*
 * ==============================================================================
 * Project: MARA Digital Twin - Motor Control
 * Sketch:  ALLMOTOR_MAGENC_PID.ino
 * Description: Multi-motor PID position control with gravity compensation.
 *              - Motor A (Big Motor 1): AS5600 Magnetic Encoder on Analog Pin A8
 *              - Motor D (Big Motor 2): AS5600 Magnetic Encoder on Analog Pin A9
 *              - Motors B, C, E: Incremental Optical/Hall Encoders
 * ==============================================================================
 */

#include <Arduino.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================

// --- AS5600 Analog Encoder Pins ---
const uint8_t AS5600_PIN_A = A8; // Motor A magnetic encoder
const uint8_t AS5600_PIN_D = A9; // Motor D magnetic encoder

// --- Motor A Driver Pins (Big Motor 1) ---
const uint8_t PWM_A = 4;
const uint8_t IN1_A = 22;
const uint8_t IN2_A = 23;

// --- Motor B Driver & Encoder Pins ---
const uint8_t PWM_B = 5;
const uint8_t IN1_B = 24;
const uint8_t IN2_B = 25;
const uint8_t ENCA_B = 18;
const uint8_t ENCB_B = 31;

// --- Motor C Driver & Encoder Pins ---
const uint8_t PWM_C = 6;
const uint8_t IN1_C = 26;
const uint8_t IN2_C = 27;
const uint8_t ENCA_C = 19;
const uint8_t ENCB_C = 33;

// --- Motor D Driver Pins (Big Motor 2) ---
const uint8_t PWM_D = 7;
const uint8_t IN1_D = 28;
const uint8_t IN2_D = 29;

// --- Motor E Driver & Encoder Pins ---
const uint8_t PWM_E = 8;
const uint8_t IN1_E = 30;
const uint8_t IN2_E = 32;
const uint8_t ENCA_E = 20;
const uint8_t ENCB_E = 35;

// ==========================================
// AS5600 MULTI-TURN TRACKING STRUCTURE
// ==========================================
struct MagEncoder {
  uint8_t pin;
  int rawPrev;
  int32_t turnCount;
  int32_t totalTicks;
  float zeroOffsetDeg;
  float currentDeg;
};

MagEncoder encA = {AS5600_PIN_A, 0, 0, 0, 0.0, 0.0};
MagEncoder encD = {AS5600_PIN_D, 0, 0, 0, 0.0, 0.0};

// Ticks per full rotation for 10-bit ADC (0 - 1023)
const float ADC_RESOLUTION = 1024.0;
const float DEG_PER_ADC_TICK = 360.0 / 1024.0;

// ==========================================
// INCREMENTAL ENCODER POSITION COUNTERS
// ==========================================
volatile int32_t pos_B = 0;
volatile int32_t pos_C = 0;
volatile int32_t pos_E = 0;

// ==========================================
// PID CONTROLLER PARAMETERS
// ==========================================
struct PIDController {
  float kp;
  float ki;
  float kd;
  float integral;
  float prevError;
  float target;
  float current;
  int pwmMin;
  int pwmMax;
};

// PID Gains tuned for MARA joints
PIDController pidA = {2.8, 0.05, 0.15, 0.0, 0.0, 0.0, 0.0, 30, 255};
PIDController pidB = {1.8, 0.02, 0.08, 0.0, 0.0, 0.0, 0.0, 30, 255};
PIDController pidC = {1.5, 0.02, 0.06, 0.0, 0.0, 0.0, 0.0, 30, 255};
PIDController pidD = {2.5, 0.04, 0.12, 0.0, 0.0, 0.0, 0.0, 30, 255};
PIDController pidE = {1.2, 0.01, 0.05, 0.0, 0.0, 0.0, 0.0, 25, 200};

// ==========================================
// GRAVITY COMPENSATION CONSTANTS
// ==========================================
const float K_GRAV_A = 0.0;   // Base joint (typically negligible gravity)
const float K_GRAV_B = 35.0;  // Shoulder joint
const float K_GRAV_C = 25.0;  // Elbow joint
const float K_GRAV_D = 20.0;  // Wrist pitch
const float K_GRAV_E = 10.0;  // Wrist roll/yaw

// ==========================================
// FUNCTION PROTOTYPES
// ==========================================
void updateMagEncoder(MagEncoder &enc);
void isr_encB();
void isr_encC();
void isr_encE();
float computePID(PIDController &pid, float dt, float feedforward);
void setMotorOutput(uint8_t pwmPin, uint8_t in1, uint8_t in2, float controlSignal);
void parseSerialCommands();

// Loop Timing
unsigned long prevTimeMicros = 0;
const unsigned long CONTROL_PERIOD_US = 10000; // 100 Hz (10 ms)

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // --- Motor Pin Modes ---
  pinMode(PWM_A, OUTPUT); pinMode(IN1_A, OUTPUT); pinMode(IN2_A, OUTPUT);
  pinMode(PWM_B, OUTPUT); pinMode(IN1_B, OUTPUT); pinMode(IN2_B, OUTPUT);
  pinMode(PWM_C, OUTPUT); pinMode(IN1_C, OUTPUT); pinMode(IN2_C, OUTPUT);
  pinMode(PWM_D, OUTPUT); pinMode(IN1_D, OUTPUT); pinMode(IN2_D, OUTPUT);
  pinMode(PWM_E, OUTPUT); pinMode(IN1_E, OUTPUT); pinMode(IN2_E, OUTPUT);

  // --- Analog Magnetic Encoder Pins ---
  pinMode(AS5600_PIN_A, INPUT);
  pinMode(AS5600_PIN_D, INPUT);

  // --- Incremental Encoder Pins ---
  pinMode(ENCA_B, INPUT_PULLUP); pinMode(ENCB_B, INPUT_PULLUP);
  pinMode(ENCA_C, INPUT_PULLUP); pinMode(ENCB_C, INPUT_PULLUP);
  pinMode(ENCA_E, INPUT_PULLUP); pinMode(ENCB_E, INPUT_PULLUP);

  // Attach Interrupts for Incremental Encoders
  attachInterrupt(digitalPinToInterrupt(ENCA_B), isr_encB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_C), isr_encC, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_E), isr_encE, RISING);

  // Initialize AS5600 readings
  analogRead(AS5600_PIN_A);
  analogRead(AS5600_PIN_D);
  delay(10);

  encA.rawPrev = analogRead(AS5600_PIN_A);
  encA.zeroOffsetDeg = encA.rawPrev * DEG_PER_ADC_TICK;

  encD.rawPrev = analogRead(AS5600_PIN_D);
  encD.zeroOffsetDeg = encD.rawPrev * DEG_PER_ADC_TICK;

  prevTimeMicros = micros();
  Serial.println(F("SYSTEM_READY: ALLMOTOR_MAGENC_PID"));
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  // Listen for target setpoints over Serial
  if (Serial.available() > 0) {
    parseSerialCommands();
  }

  unsigned long currentTimeMicros = micros();
  if (currentTimeMicros - prevTimeMicros >= CONTROL_PERIOD_US) {
    float dt = (currentTimeMicros - prevTimeMicros) / 1000000.0f;
    prevTimeMicros = currentTimeMicros;

    // 1. Update AS5600 Magnetic Encoders (Motors A & D)
    updateMagEncoder(encA);
    updateMagEncoder(encD);

    // Current positions in degrees
    pidA.current = encA.currentDeg;
    pidB.current = pos_B * 0.1f; // Adjust conversion factor based on your gear ratio/PPR
    pidC.current = pos_C * 0.1f;
    pidD.current = encD.currentDeg;
    pidE.current = pos_E * 0.1f;

    // 2. Compute Gravity Compensation (Feedforward torque)
    // joint angles in radians for trigonometric calculation
    float radB = radians(pidB.current);
    float radC = radians(pidC.current);
    float radD = radians(pidD.current);

    float gravCompA = 0.0;
    float gravCompB = K_GRAV_B * cos(radB);
    float gravCompC = K_GRAV_C * cos(radB + radC);
    float gravCompD = K_GRAV_D * cos(radB + radC + radD);
    float gravCompE = 0.0;

    // 3. Compute PID Outputs with Feedforward
    float outA = computePID(pidA, dt, gravCompA);
    float outB = computePID(pidB, dt, gravCompB);
    float outC = computePID(pidC, dt, gravCompC);
    float outD = computePID(pidD, dt, gravCompD);
    float outE = computePID(pidE, dt, gravCompE);

    // 4. Drive Motors
    setMotorOutput(PWM_A, IN1_A, IN2_A, outA);
    setMotorOutput(PWM_B, IN1_B, IN2_B, outB);
    setMotorOutput(PWM_C, IN1_C, IN2_C, outC);
    setMotorOutput(PWM_D, IN1_D, IN2_D, outD);
    setMotorOutput(PWM_E, IN1_E, IN2_E, outE);

    // 5. Telemetry output
    static uint8_t telemetryDiv = 0;
    if (++telemetryDiv >= 10) { // 10 Hz telemetry
      telemetryDiv = 0;
      Serial.print(F("FB:"));
      Serial.print(pidA.current, 2); Serial.print(F(","));
      Serial.print(pidB.current, 2); Serial.print(F(","));
      Serial.print(pidC.current, 2); Serial.print(F(","));
      Serial.print(pidD.current, 2); Serial.print(F(","));
      Serial.println(pidE.current, 2);
    }
  }
}

// ==========================================
// AS5600 ANALOG ENCODER UNWRAPPING
// ==========================================
void updateMagEncoder(MagEncoder &enc) {
  // Read analog value (0 - 1023)
  int raw = analogRead(enc.pin);
  int diff = raw - enc.rawPrev;

  // Detect and handle wrap-around across the 0 <-> 1023 boundary
  if (diff > 512) {
    enc.turnCount--;
  } else if (diff < -512) {
    enc.turnCount++;
  }

  enc.rawPrev = raw;
  enc.totalTicks = (enc.turnCount * 1024) + raw;
  enc.currentDeg = (enc.totalTicks * DEG_PER_ADC_TICK) - enc.zeroOffsetDeg;
}

// ==========================================
// PID COMPUTATION FUNCTION
// ==========================================
float computePID(PIDController &pid, float dt, float feedforward) {
  float error = pid.target - pid.current;

  // Proportional
  float pTerm = pid.kp * error;

  // Integral with anti-windup clamping
  pid.integral += error * dt;
  float iTerm = pid.ki * pid.integral;
  iTerm = constrain(iTerm, -100.0f, 100.0f);

  // Derivative
  float dTerm = 0.0;
  if (dt > 0.0f) {
    dTerm = pid.kd * (error - pid.prevError) / dt;
  }
  pid.prevError = error;

  // Total Control Output
  float totalOut = pTerm + iTerm + dTerm + feedforward;
  return totalOut;
}

// ==========================================
// MOTOR DRIVER OUTPUT
// ==========================================
void setMotorOutput(uint8_t pwmPin, uint8_t in1, uint8_t in2, float controlSignal) {
  int pwmValue = abs((int)controlSignal);

  // Apply deadband and limit to max PWM
  if (pwmValue > 255) pwmValue = 255;
  if (pwmValue < 15 && pwmValue > 0) pwmValue = 0; // Deadband

  if (controlSignal > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, pwmValue);
  } else if (controlSignal < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, pwmValue);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

// ==========================================
// SERIAL COMMAND PARSER
// Format: "T:tgtA,tgtB,tgtC,tgtD,tgtE\n"
// ==========================================
void parseSerialCommands() {
  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.startsWith("T:")) {
    input = input.substring(2); // Remove "T:"
    
    float tA, tB, tC, tD, tE;
    int parsed = sscanf(input.c_str(), "%f,%f,%f,%f,%f", &tA, &tB, &tC, &tD, &tE);
    if (parsed == 5) {
      pidA.target = tA;
      pidB.target = tB;
      pidC.target = tC;
      pidD.target = tD;
      pidE.target = tE;
    }
  } else if (input.equalsIgnoreCase("ZERO")) {
    encA.zeroOffsetDeg = (encA.turnCount * 1024 + encA.rawPrev) * DEG_PER_ADC_TICK;
    encD.zeroOffsetDeg = (encD.turnCount * 1024 + encD.rawPrev) * DEG_PER_ADC_TICK;
    noInterrupts();
    pos_B = 0;
    pos_C = 0;
    pos_E = 0;
    interrupts();
    Serial.println(F("ZERO_OK"));
  }
}

// ==========================================
// INTERRUPT SERVICE ROUTINES (Motors B, C, E)
// ==========================================
void isr_encB() {
  if (digitalRead(ENCB_B) == HIGH) {
    pos_B++;
  } else {
    pos_B--;
  }
}

void isr_encC() {
  if (digitalRead(ENCB_C) == HIGH) {
    pos_C++;
  } else {
    pos_C--;
  }
}

void isr_encE() {
  if (digitalRead(ENCB_E) == HIGH) {
    pos_E++;
  } else {
    pos_E--;
  }
}