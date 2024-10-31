#include <Servo.h>

Servo monServo;       // First servo motor
Servo secondServo;    // Second servo motor

#define echoPin 2
#define trigPin 3
const int buzzer = 6;  // Buzzer pin

long duration;         // Time for the echo to return
int distance;          // Calculated distance

bool isObjectDetected = false;  // State variable for object detection

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzer, OUTPUT);

  monServo.attach(5);
  secondServo.attach(4);
  monServo.write(0);
  secondServo.write(0);
  delay(500);
}

void loop() {
  distance = getDistance();

  // Print distance reading
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= 5 && !isObjectDetected) {
    isObjectDetected = true;
    activateBuzzer(true);

    // Print object detection status and buzzer state
    Serial.println("Object detected within 5 cm!");
    Serial.println("Buzzer: ON");

    oscillateServos();
  } else if (distance > 5 && isObjectDetected) {
    isObjectDetected = false;
    activateBuzzer(false);

    // Print buzzer state when it turns off
    Serial.println("No object detected within 5 cm.");
    Serial.println("Buzzer: OFF");

    resetServos();
  }

  delay(500);
}

int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void activateBuzzer(bool state) {
  digitalWrite(buzzer, state ? HIGH : LOW);
}

void oscillateServos() {
  for (int pos = 0; pos <= 180; pos++) {
    monServo.write(pos);
    secondServo.write(pos);
    delay(10);  // Adjust the speed of movement here
  }
  for (int pos = 180; pos >= 0; pos--) {
    monServo.write(pos);
    secondServo.write(pos);
    delay(10);
  }
}

void resetServos() {
  monServo.write(0);
  secondServo.write(0);
}



