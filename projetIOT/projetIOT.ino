#include <DHT.h>
#include <Servo.h>

Servo monServo;    // First servo motor
Servo secondServo; // Second servo motor

#define echoPin 2
#define trigPin 3
const int buzzer = 6;  // Buzzer pin

long duration;         // Time for the echo to return
int distance;          // Calculated distance

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
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);        
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);       
  digitalWrite(trigPin, LOW);  

  duration = pulseIn(echoPin, HIGH);  
  distance = duration * 0.034 / 2;    

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= 5) {
    activateBuzzer(true);
    moveServos(0, 190);
    moveServos(190, 0);
  } else {
    activateBuzzer(false);
    moveServos(0, 0);
  }

  delay(500);  
}

void activateBuzzer(bool state) {
  digitalWrite(buzzer, state ? HIGH : LOW);
}

void moveServos(int startPos, int endPos) {
  for (int pos = startPos; pos <= endPos; pos++) {  
    monServo.write(pos);
    secondServo.write(pos); 
    delay(1);  
  }
}


