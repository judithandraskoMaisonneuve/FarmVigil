#include <Servo.h>

// Servo motors
Servo monServo;
Servo secondServo;

// Pins
#define echoPin 2
#define trigPin 3
const int buzzer = 6;
const int ledPin = 8;

// Variables
long duration;
float distance;
bool isObjectDetected = false; // State for object detection
bool buzzer_state = false;     // State for buzzer activation

void setup() {
  Serial.begin(9600);

  // Pin modes
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // Attach and initialize servos
  monServo.attach(5);
  secondServo.attach(4);
  monServo.write(0);
  secondServo.write(0);
  delay(500);
}

void loop() {
  // Measure distance
  distance = measureDistance();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Object detection logic
  if (distance <= 5 && !isObjectDetected) {
    isObjectDetected = true;
    activateBuzzer(true);

    // Print object detection status
    Serial.println("Object detected within 5 cm!");
    Serial.println("Buzzer: ON");

    oscillateServos();
  } else if (distance > 5 && isObjectDetected) {
    isObjectDetected = false;
    activateBuzzer(false);

    // Print status when no object is detected
    Serial.println("No object detected within 5 cm.");
    Serial.println("Buzzer: OFF");

    resetServos();
  }

  // Handle communication with Raspberry Pi
  if (Serial.available() > 0) {
    char response = Serial.read();

    if (response == 'N') { // No person detected
      activateBuzzer(true);
      oscillateServos();
    } else if (response == 'P') { // Person detected
      activateBuzzer(false);
      resetServos();
      digitalWrite(ledPin, LOW);
    } else if (response == 'L') { // Light control
      digitalWrite(ledPin, HIGH);
    }
  }

  delay(500);
}

float measureDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout

    if (duration == 0) {
        return 5; // No valid distance
    }

    float distance = duration * 0.034 / 2;

    // Ignore unrealistic readings
    if (distance < 2 || distance > 400) { // Sensor range is typically 2-400 cm
        return -1; // Invalid distance
    }

    return distance;
}


void activateBuzzer(bool state) {
  buzzer_state = state;
  digitalWrite(buzzer, state ? HIGH : LOW);
}

void oscillateServos() {
  // Oscillate servos between 0 and 180 degrees
  for (int pos = 0; pos <= 180; pos++) {
    monServo.write(pos);
    secondServo.write(pos);
    delay(10); // Adjust movement speed
  }
  for (int pos = 180; pos >= 0; pos--) {
    monServo.write(pos);
    secondServo.write(pos);
    delay(10);
  }
}

void resetServos() {
  // Reset servos to initial position
  monServo.write(0);
  secondServo.write(0);
}
