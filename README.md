
pin descriptions 
---seed protection module---
trigPin = 2
echoPin = 3
buzzerPin = A1
---nav module---
motor1Pin1 = 8    // IN1 - Left motor pin 1
motor1Pin2 = 9    // IN2 - Left motor pin 2
motor2Pin1 = 10   // IN3 - Right motor pin 1
motor2Pin2 = 11   // IN4 - Right motor pin 2
enaPin = 5        // ENA - Left motor speed
enbPin = 6 
---Moisture Sensor module---
const int moistureSensorPin = A0; // YL-69 analog pin
const int DRY_VALUE = 1023;          // dry reading
const int WET_VALUE = 0;             // wet reading
const int MOISTURE_THRESHOLD = 20;    // percentage threshold to trigger
const int MOTOR_SPEED = 150;         // PWM speed (0-255)
---Servo module---
Servo hookServo;      // pin 12, start 180°, "lower" to 120°
Servo flapperServo;   // pin 4,  start 180°, "lower" to 120°
Servo dispenserServo; // pin 13, idle 0°,   "dispense" 0<->90 oscillation
