#include <Arduino.h>

#define IN1 23
#define IN2 24
#define PWM_PIN 4 
#define ENC_A 2  
#define ENC_B 3  

#define COUNTS_PER_REV 4532 //check again if it's 1133 or 1133*4

volatile long encoderCount = 0;

unsigned long lastMicros = 0;
unsigned long startMicros = 0;

int pwmValue = 0;
const int PWM_MIN = 0;
const int PWM_MAX = 250;
const int PWM_STEP = 10;

const unsigned long STEP_TIME_MS = 5000UL; 

unsigned long lastStepMillis = 0;
int pwmDir = 1;
bool running = false;
bool finished = false;

// modify for our motor, can we use quadrature??????
void encoderISR() {
  static uint8_t enc_val = 0;
  enc_val = (enc_val << 2) | ((digitalRead(ENC_A) << 1) | digitalRead(ENC_B));
  
  switch (enc_val & 0b1111) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCount++; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: encoderCount--; break;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

  analogWrite(PWM_PIN, 0);

  Serial.println("time_us,pwm,ticks");
}

void loop() {
  if (finished) {
    analogWrite(PWM_PIN, 0);
    return;
  }

  // --- Wait for 'f' or 'F' to start ---
  if (!running) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'f' || c == 'F') {
        running = true;
        pwmValue = PWM_MIN;
        pwmDir = 1;
        digitalWrite(IN1, HIGH); // Set initial direction
        digitalWrite(IN2, LOW);
        analogWrite(PWM_PIN, pwmValue);
        
        noInterrupts(); 
        encoderCount = 0; // Reset position to 0 ONLY at the very start
        interrupts();
        
        startMicros = micros();
        lastMicros = startMicros;
        lastStepMillis = millis();
      }
    }
    return;
  }

  // ---------- TICK LOGGING (Every 100ms) ----------
  unsigned long nowMicros = micros();
  unsigned long dt = nowMicros - lastMicros;

  if (dt >= 100000UL) { // 100ms sampling rate (DT = 0.1s in Python)
    noInterrupts();
    long safeTicks = encoderCount;
    interrupts();

    unsigned long t_rel = nowMicros - startMicros;

    Serial.print(t_rel);
    Serial.print(",");
    Serial.print(pwmValue);
    Serial.print(",");
    Serial.println(safeTicks);

    lastMicros = nowMicros;
  }

  // ---------- PWM STAIRCASE LOGIC ----------
  if (millis() - lastStepMillis >= STEP_TIME_MS) {
    int next = pwmValue + pwmDir * PWM_STEP;
    
    if (pwmDir > 0) { // Stepping UP
      if (next >= PWM_MAX) { 
        pwmValue = PWM_MAX; 
        pwmDir = -1; // Turn around and go down
        digitalWrite(IN1, LOW); // Change direction
        digitalWrite(IN2, HIGH);
      } else { 
        pwmValue = next; 
      }
    } else { // Stepping DOWN
      if (next <= PWM_MIN) { 
        pwmValue = PWM_MIN; 
        finished = true; // Stop when we hit the bottom
        analogWrite(PWM_PIN, 0);
        return; 
      } else { 
        pwmValue = next; 
      }
    }
    analogWrite(PWM_PIN, pwmValue);
    lastStepMillis = millis();
  }
}