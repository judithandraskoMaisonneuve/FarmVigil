#include <DHT.h>
#include <Servo.h>

Servo monServo;

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
  pinMode(buzzer, OUTPUT);   // Pin for the buzzer

  monServo.attach(5);        // Attach the servo to pin 5
  monServo.write(0);         // Set the servo to 0 degrees
  delay(500);               // Wait 1 second
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

  // If the distance is 5 cm or less, activate the buzzer and move the servo between 0 and 190 degrees continuously
  if (distance <= 5) {
    digitalWrite(buzzer, HIGH);  // Turn on the buzzer
    
    // Make the servo motor move continuously between 0 and 180 degrees faster
    for (int pos = 0; pos <= 190; pos += 1) {  // Move the servo from 0 to 190 degrees
      monServo.write(pos);
      delay(1);  // Faster movement (reduced delay from 15ms to 5ms)
    }
    for (int pos = 190; pos >= 0; pos -= 1) {  // Move the servo back from 190 to 0 degrees
      monServo.write(pos);
      delay(1);  // Fast mouvement
    }
    
  } else {
    digitalWrite(buzzer, LOW);   // Turn off the buzzer
    monServo.write(0);           // Move the servo back to 0 degrees
  }

  delay(500);  // Wait half a second before the next reading
}
