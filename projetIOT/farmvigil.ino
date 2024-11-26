#include <Servo.h>

Servo monServo;
Servo secondServo;

#define echoPin 2
#define trigPin 3
const int buzzer = 6;

long duration;
float distance;
bool buzzer_state = false;  // Track buzzer state

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(8, OUTPUT);
  monServo.attach(5);
  secondServo.attach(4);
  monServo.write(0);
  secondServo.write(0);
}

void loop() {

  distance = getDistance();
  Serial.println(distance);   

    // Wait for response from Raspberry Pi
  if (Serial.available() > 0) {
    char response = Serial.read();

      
    if (response == 'N') {  // 'N' for no person detected
      setBuzzer(true);
      oscillateServos();  // Oscillate servos if no person is detected
    } else if (response == 'P') {  // 'P' for person detected
      setBuzzer(false);
      resetServos();
      digitalWrite(8,LOW);
    } else if (response == 'L'){
        digitalWrite(8,HIGH);
      }
    }
  

  delay(1000);  // Adjust for suitable refresh rate
}

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);  // Timeout of 30ms
  return duration * 0.034 / 2;
}

void setBuzzer(bool state) {
  buzzer_state = state;
  digitalWrite(buzzer, state ? HIGH : LOW);
}

void oscillateServos() {
  // Oscillate both servos between 0 and 180 degrees
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
  // Reset servos to the starting position
  monServo.write(0);
  secondServo.write(0);
}
