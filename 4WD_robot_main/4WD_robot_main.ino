// === Combined Robot Sketch (Updated) ===
// Fixes:
// 1) Buzzer only buzzes when ultrasonic detects an object within 30 cm, and
//    distance is checked continuously during operation. Buzz for 3 seconds per detection.
// 2) While moving backward, servo_dispenser oscillates between 0° and 90° at ~3 ms intervals.
//
// Pin definitions preserved.

#include <Servo.h>

// --- From buzzer_module.ino ---
const int trigPin = 2;
const int echoPin = 3;
const int buzzerPin = A1;

// --- From yl_69_sensor_movement_test.ino ---
const int motor1Pin1 = 8;    // IN1 - Left motor pin 1
const int motor1Pin2 = 9;    // IN2 - Left motor pin 2
const int motor2Pin1 = 10;   // IN3 - Right motor pin 1
const int motor2Pin2 = 11;   // IN4 - Right motor pin 2
const int enaPin = 5;        // ENA - Left motor speed
const int enbPin = 6;        // ENB - Right motor speed

const int moistureSensorPin = A0; // YL-69 analog pin

// Calibrations / controls (preserved)
const int DRY_VALUE = 1023;          // dry reading
const int WET_VALUE = 0;             // wet reading
const int MOISTURE_THRESHOLD = 20;    // percentage threshold to trigger
const int MOTOR_SPEED = 150;         // PWM speed (0-255)

// --- From servo files ---
Servo hookServo;      // pin 12, start 180°, "lower" to 120°
Servo flapperServo;   // pin 4,  start 180°, "lower" to 120°
Servo dispenserServo; // pin 13, idle 0°,   "dispense" 0<->90 oscillation

// ---------------- Utility: Motors ----------------
void goForward() {
  // Left motor forward
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(enaPin, MOTOR_SPEED);

  // Right motor forward
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enbPin, MOTOR_SPEED);
}

void goBackward() {
  // Left motor backward
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(enaPin, MOTOR_SPEED);

  // Right motor backward
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(enbPin, MOTOR_SPEED);
}

void stopCar() {
  // Stop left
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(enaPin, 0);
  // Stop right
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enbPin, 0);
}

// ---------------- Utility: Moisture ----------------
int readMoisturePercent() {
  int raw = analogRead(moistureSensorPin);
  int pct = map(raw, DRY_VALUE, WET_VALUE, 0, 100);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

// ---------------- Utility: Ultrasonic & Bird Guard ----------------
long measureDistanceCm() {
  // Trigger the ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo (timeout ~30ms => ~5m max)
  long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return 999; // no echo
  long cm = (long)(duration * 0.034f / 2.0f);
  return cm;
}

// Non-blocking buzzer controller: buzzes for 3s when an object is <= 30cm
void updateBirdGuard() {
  static bool buzzing = false;
  static unsigned long buzzEndMs = 0;
  static unsigned long lastMeasureMs = 0;

  unsigned long now = millis();

  // End buzzing after 3 seconds
  if (buzzing && now >= buzzEndMs) {
    noTone(buzzerPin);
    buzzing = false;
  }

  // Limit distance sampling to ~60 ms intervals
  if (now - lastMeasureMs >= 60) {
    lastMeasureMs = now;
    long dcm = measureDistanceCm();
    // Uncomment to see live distance:
    // Serial.print("Dist: "); Serial.print(dcm); Serial.println(" cm");

    if (!buzzing && dcm > 0 && dcm <= 30) {
      // Start buzzing
      tone(buzzerPin, 1000);
      buzzing = true;
      buzzEndMs = now + 3000UL;
      Serial.println("Bird detected! Buzz for 3s.");
    }
  }
}

// ---------------- Functions (converted from separate files) ----------------

// Hook servo controls (from servo_hook.ino)
void hook_set_start() {        // 180°
  hookServo.write(180);
  delay(200);
}

void hook_lower() {            // 120°
  hookServo.write(120);
  delay(200);
}

void hook_raise() {            // 180°
  hookServo.write(180);
  delay(200);
}

// Flapper servo controls (from servo_flapper.ino)
void flapper_set_start() {     // 180°
  flapperServo.write(180);
  delay(200);
}

void flapper_lower() {         // 120°
  flapperServo.write(120);
  delay(200);
}

void flapper_raise() {         // 180°
  flapperServo.write(180);
  delay(200);
}

// Dispenser servo action (updated to fast oscillation control in backward move)
void dispenser_to_0() { dispenserServo.write(0); }
void dispenser_to_90() { dispenserServo.write(90); }

// ---------------- Movement helpers with continuous checks ----------------
void moveForwardFor(unsigned long ms) {
  unsigned long t0 = millis();
  goForward();
  while (millis() - t0 < ms) {
    updateBirdGuard();
    delay(2); // small yield
  }
  stopCar();
}

void moveBackwardForWithDispense(unsigned long ms) {
  unsigned long t0 = millis();
  unsigned long lastToggle = millis();
  bool at90 = false;
  goBackward();
  // initialize dispenser to 0 at start
  dispenser_to_0();

  while (millis() - t0 < ms) {
    updateBirdGuard();

    // Oscillate 0 <-> 90 every ~3 ms
    if (millis() - lastToggle >= 3) {
      if (at90) {
        dispenser_to_0();
      } else {
        dispenser_to_90();
      }
      at90 = !at90;
      lastToggle = millis();
    }

    // small yield to avoid starving other tasks
    delay(1);
  }

  stopCar();
  dispenser_to_0(); // return to 0° at end
}

// ---------------- Setup & Loop ----------------
void setup() {
  Serial.begin(9600);

  // Buzzer/ultrasonic pins (kept same)
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);

  // Motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enaPin, OUTPUT);
  pinMode(enbPin, OUTPUT);

  // Servos (kept same pins)
  hookServo.attach(12);       // servo_hook.ino
  flapperServo.attach(4);     // servo_flapper.ino
  dispenserServo.attach(13);  // servo_dispenser.ino

  // Initialize servos to their "start" positions per originals
  hook_set_start();           // 180°
  flapper_set_start();        // 180°
  dispenser_to_0();           // idle 0°

  stopCar();
}

void loop() {
  // 1) Wait until soil moisture percentage >= 5 (per yl_69_sensor_movement_test.ino)
  int moisturePct = readMoisturePercent();
  Serial.print("Moisture: ");
  Serial.print(moisturePct);
  Serial.println("%");

  while (moisturePct < MOISTURE_THRESHOLD) {
    updateBirdGuard();   // keep bird guard active even while waiting
    delay(200);
    moisturePct = readMoisturePercent();
    Serial.print("Moisture: ");
    Serial.print(moisturePct);
    Serial.println("%");
  }

  // Optional settle time can be inserted here if desired:
   delay(4000);

  // 2) Lower the hook servo (servo_hook angles)
  hook_lower();

  // 3) Move forward for 5 seconds with continuous bird-guard checking
  moveForwardFor(5000UL);

  // 4) Retract the hook servo
  hook_raise();

  // 5) Lower the flapper servo (servo_flapper angles)
  flapper_lower();

  // 6) Move backward for 5 seconds, with dispenser oscillating at ~3 ms
  moveBackwardForWithDispense(5000UL);

  // 7) Return flapper to original position
  flapper_raise();

  // Short pause before next cycle
  delay(1000);
}
