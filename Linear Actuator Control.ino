#include <Arduino.h>

// Define pin connections for the first actuator
const int enablePin1 = 9;
const int input1Pin1 = 8;
const int input2Pin1 = 6;

// Define pin connections for the second actuator
const int enablePin2 = 10; // Example pin, adjust as needed
const int input1Pin2 = 12; // Example pin, adjust as needed
const int input2Pin2 = 11; // Example pin, adjust as needed

// Define the extension and retraction time (in milliseconds)
const unsigned long fullExtensionTime = 8666; // 10 seconds  15MM/s 130mm stroke 64N >> 8.67s total
// 2888.66 > 0 deg
// 1 deg: 96.28
const unsigned long neutralExtensionTime = fullExtensionTime * 69/90;
const unsigned long neutralRetractionTime = fullExtensionTime * 72/90;

const unsigned long partialExtensionTime = fullExtensionTime * 0.50; // 90% of full extension time

void setup() {
  // Set the first actuator control pins as outputs; right wheel
  pinMode(enablePin1, OUTPUT);
  pinMode(input1Pin1, OUTPUT);
  pinMode(input2Pin1, OUTPUT);

  // Set the second actuator control pins as outputs; left wheel
  pinMode(enablePin2, OUTPUT); 
  pinMode(input1Pin2, OUTPUT);
  pinMode(input2Pin2, OUTPUT);

  Serial.begin(115200);
  Serial.println("Arduino is ready.");
  delay(2000);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    // Debug: Print the received command
    Serial.print("Received command: ");
    Serial.println(command);
    
    Serial.flush();  // Clear the buffer to ensure no leftover data
    
    if (command == '1') {
      // Retract both actuators for the full extension time (if needed)
      digitalWrite(input1Pin1, LOW);
      digitalWrite(input2Pin1, HIGH);
      analogWrite(enablePin1, 145);  // Maximum PWM value

      digitalWrite(input1Pin2, LOW);
      digitalWrite(input2Pin2, HIGH);
      analogWrite(enablePin2, 255);  // Maximum PWM value

      delay(neutralRetractionTime);  // Wait for full retraction
      analogWrite(enablePin1, 0);  // Stop the actuators
      analogWrite(enablePin2, 0);  // Stop the actuators

      Serial.println("Retracting actuators to full position");
    } else if (command == '0') {
      // Extend both actuators for the partial extension time (90%)
      digitalWrite(input1Pin1, HIGH);
      digitalWrite(input2Pin1, LOW);
      analogWrite(enablePin1, 145);  // Maximum PWM value

      digitalWrite(input1Pin2, HIGH);
      digitalWrite(input2Pin2, LOW);
      analogWrite(enablePin2, 255);  // Maximum PWM value

      delay(neutralRetractionTime);  // Wait for 90% extension
      analogWrite(enablePin1, 0);  // Stop the actuators
      analogWrite(enablePin2, 0);  // Stop the actuators

      Serial.println("Extending actuators to 90% position");
    } else {
      Serial.println("Unknown command");
    }
  }
}
