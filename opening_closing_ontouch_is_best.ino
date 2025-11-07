#define DIR_PIN 2
#define STEP_PIN 4
#define STEPS_PER_REV 200

#define TRIG_PIN 5
#define ECHO_PIN 18

#define TOUCH_BOTTOM_LEFT 38
#define TOUCH_BOTTOM_RIGHT 36
#define TOUCH_FACE_LEFT 37
#define TOUCH_FACE_RIGHT 35

enum State { 
  MOTOR_STOPPED,           // Step 0: Motor stopped, waiting for first touch
  ANTICLOCK_ROTATE,        // Step 1: Rotate anticlockwise until both face sensors touch
  WAIT_1MIN_AFTER_ANTI,    // Wait 1 min after anticlockwise
  CLOCK_ROTATE,            // Step 2: Rotate clockwise for 2 complete rounds
  WAIT_1MIN_AFTER_CLOCK    // Wait 1 min after clockwise
};
State state = MOTOR_STOPPED;

unsigned long faceTouchStartTime = 0;
unsigned long lastActionTime = 0;
int clockTurns = 0;

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(TOUCH_BOTTOM_LEFT, INPUT);
  pinMode(TOUCH_BOTTOM_RIGHT, INPUT);
  pinMode(TOUCH_FACE_LEFT, INPUT);
  pinMode(TOUCH_FACE_RIGHT, INPUT);

  Serial.begin(115200);
  Serial.println("Gripper system ready. Motor stopped - waiting for first touch.");
}

void loop() {
  long distance = getDistance();
  bool touchBottom = (digitalRead(TOUCH_BOTTOM_LEFT) == HIGH) || (digitalRead(TOUCH_BOTTOM_RIGHT) == HIGH);
  bool touchFace = (digitalRead(TOUCH_FACE_LEFT) == HIGH) && (digitalRead(TOUCH_FACE_RIGHT) == HIGH);
  unsigned long now = millis();

  // Debug print
  Serial.print("State: "); Serial.print(state);
  Serial.print(" | Bottom: "); Serial.print(touchBottom ? "Y" : "N");
  Serial.print(" | Face: "); Serial.print(touchFace ? "Y" : "N");
  Serial.print(" | Dist: "); Serial.println(distance);

  switch (state) {
    case MOTOR_STOPPED:
      // Step 0: Motor is stopped, waiting for first touch
      if (touchBottom) {
        Serial.println("Bottom touch detected. Starting ANTICLOCKWISE rotation.");
        digitalWrite(DIR_PIN, LOW); // LOW for anticlockwise
        state = ANTICLOCK_ROTATE;
      }
      break;

    case ANTICLOCK_ROTATE:
      // Step 1: Rotate anticlockwise until both face sensors touch
      if (touchFace) {
        Serial.println("Both face touches detected. Motor stopped. Waiting 1 minute...");
        lastActionTime = now;
        state = WAIT_1MIN_AFTER_ANTI;
      } else {
        stepMotor();
      }
      break;

    case WAIT_1MIN_AFTER_ANTI:
      // Wait 1 minute after anticlockwise rotation
      if (now - lastActionTime >= 60000) { // 60000ms = 1 minute
        if (touchBottom) {
          Serial.println("1 minute passed, bottom touch active. Starting CLOCKWISE rotation for 2 rounds.");
          digitalWrite(DIR_PIN, HIGH); // HIGH for clockwise
          clockTurns = 0;
          state = CLOCK_ROTATE;
        }
      } else {
        // Show countdown
        unsigned long remaining = (60000 - (now - lastActionTime)) / 1000;
        if (remaining % 10 == 0) { // Print every 10 seconds
          Serial.print("Waiting: ");
          Serial.print(remaining);
          Serial.println(" seconds remaining...");
        }
      }
      break;

    case CLOCK_ROTATE:
      // Step 2: Rotate clockwise for 2 complete rounds
      if (clockTurns < 1) {
        // Complete one full rotation
        for (int i = 0; i < STEPS_PER_REV; i++) {
          stepMotor();
        }
        clockTurns++;
        Serial.print("Clockwise rotation count: ");
        Serial.println(clockTurns);
        delay(500); // Small pause between turns
      } else {
        Serial.println("Completed 2 clockwise rotations. Motor stopped. Waiting 1 minute...");
        lastActionTime = now;
        state = WAIT_1MIN_AFTER_CLOCK;
      }
      break;

    case WAIT_1MIN_AFTER_CLOCK:
      // Wait 1 minute after clockwise rotation
      if (now - lastActionTime >= 60000) { // 60000ms = 1 minute
        if (touchBottom) {
          Serial.println("1 minute passed, bottom touch active. Starting ANTICLOCKWISE rotation.");
          digitalWrite(DIR_PIN, LOW); // LOW for anticlockwise
          state = ANTICLOCK_ROTATE;
        }
      } else {
        // Show countdown
        unsigned long remaining = (60000 - (now - lastActionTime)) / 1000;
        if (remaining % 10 == 0) { // Print every 10 seconds
          Serial.print("Waiting: ");
          Serial.print(remaining);
          Serial.println(" seconds remaining...");
        }
      }
      break;
  }
}

// Function to step the motor once
void stepMotor() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(1000);
}

// Function to get distance from HC-SR04 in mm
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout 30ms
  long distance = duration * 0.343 / 2; // mm (speed of sound = 343 m/s)
  return distance;
}