#define IN1 23
#define IN2 24
#define PWM_PIN 4
#define ENC_A 2
#define ENC_B 22

#define SOME_LIMIT 15000 //encoder tick limit

volatile long encoderCount = 0;

unsigned long lastLogMicros = 0;
unsigned long startMicros = 0;

bool running = false;
bool finished = false;

const int STEP_PWM = 150;                //constant step
const unsigned long SAMPLE_US = 10000UL; // 0.01s
const unsigned long RUN_TIME_MS = 1500;  // 1.5s data measurement

void encoderISR() {
  if (digitalRead(ENC_B))
    encoderCount++;
  else
    encoderCount--;
}

void applyMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM_PIN, pwm);
  } else if (pwm < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(PWM_PIN, -pwm);
  } else {
    analogWrite(PWM_PIN, 0);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  Serial.println("time_us,pwm,ticks");
}

void loop() {
  if (finished) {
    analogWrite(PWM_PIN, 0);
    return;
  }

  if (!running) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'f' || c == 'F') {
        running = true;

        noInterrupts();
        encoderCount = 0;
        interrupts();

        startMicros = micros();
        lastLogMicros = startMicros;
        
        // Inject the constant step exactly once
        applyMotor(STEP_PWM); 
      }
    }
    return;
  }

//stop after passing run time
  if (micros() - startMicros >= (RUN_TIME_MS * 1000UL)) {
    applyMotor(0);
    finished = true;
    Serial.println("TIME LIMIT REACHED");
    return;
  }

//limits the encoder ticks
  noInterrupts();
  long safeCount = encoderCount;
  interrupts();

  if (abs(safeCount) > SOME_LIMIT) {
    applyMotor(0);
    finished = true;
    Serial.println("LIMIT REACHED");
    return;
  }

//printing for data
  unsigned long now = micros();

  if (now - lastLogMicros >= SAMPLE_US) {
    noInterrupts();
    long ticks = encoderCount;
    interrupts();

    Serial.print(now - startMicros);
    Serial.print(",");
    Serial.print(STEP_PWM);
    Serial.print(",");
    Serial.println(ticks);

    lastLogMicros = now;
  }
}