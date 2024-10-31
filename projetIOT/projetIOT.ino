#include <DHT.h>
#include <Servo.h>

Servo monServo;    // First servo motor
Servo secondServo; // Second servo motor

#define echoPin 2
#define trigPin 3

const int buzzer = 6;  // Buzzer pin
long duration;         // Variable to store the time it takes for the echo to return
int distance;          // Variable to store the calculated distance

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Setup pin modes and initial states
  pinMode(trigPin, OUTPUT);  // Pin for distance sensor output (trigger)
  pinMode(echoPin, INPUT);   // Pin for distance sensor input (echo)
  pinMode(buzzer, OUTPUT);    // Pin for the buzzer

  monServo.attach(5);         // Attach the first servo to pin 5
  secondServo.attach(4);      // Attach the second servo to pin 4
  monServo.write(0);          // Set the first servo to 0 degrees
  secondServo.write(0);       // Set the second servo to 0 degrees
  delay(500);                 // Wait half a second
}

void loop() {
  // Calculate the distance from the ultrasonic sensor
  digitalWrite(trigPin, LOW);  // Clear the trigger
  delayMicroseconds(2);        
  digitalWrite(trigPin, HIGH); // Send a trigger pulse
  delayMicroseconds(10);       
  digitalWrite(trigPin, LOW);  

  duration = pulseIn(echoPin, HIGH);  // Measure the time of the echo
  distance = duration * 0.034 / 2;    // Convert time to distance (in cm)

  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // If the distance is 5 cm or less, activate the buzzer and move the servos
  if (distance <= 5) {
    digitalWrite(buzzer, HIGH);  // Turn on the buzzer
    
    // Move both servos between 0 and 190 degrees
    for (int pos = 0; pos <= 190; pos += 1) {  
      monServo.write(pos);
      secondServo.write(pos); // Move the second servo simultaneously
      delay(1);  // Fast movement
    }
    for (int pos = 190; pos >= 0; pos -= 1) {  
      monServo.write(pos);
      secondServo.write(pos); // Move the second servo simultaneously
      delay(1);  // Fast movement
    }
    
  } else {
    digitalWrite(buzzer, LOW);   // Turn off the buzzer
    monServo.write(0);           // Move the first servo back to 0 degrees
    secondServo.write(0);        // Move the second servo back to 0 degrees
  }

  delay(500);  // Wait half a second before the next reading
}

