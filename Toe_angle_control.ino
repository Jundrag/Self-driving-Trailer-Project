#include <Arduino.h>

// Define pin connections for the first actuator
const int enablePin1 = 9;
const int input1Pin1 = 8;
const int input2Pin1 = 7;

// Define pin connections for the second actuator
const int enablePin2 = 10; // Example pin, adjust as needed
const int input1Pin2 = 12; // Example pin, adjust as needed
const int input2Pin2 = 11; // Example pin, adjust as needed

void setup() {
  // Set the first actuator control pins as outputs
  pinMode(enablePin1, OUTPUT);
  pinMode(input1Pin1, OUTPUT);
  pinMode(input2Pin1, OUTPUT);

  // Set the second actuator control pins as outputs
  pinMode(enablePin2, OUTPUT);
  pinMode(input1Pin2, OUTPUT);
  pinMode(input2Pin2, OUTPUT);

  Serial.begin(115200);
  Serial.println("Arduino is ready.");
}

void loop() {
  if (Serial.available() > 0) {
    //int command = Serial.parseInt();
    char command = Serial.read();

    // change made
    // String command = Serial.readStringUntil('\n');
    // command.trim();
    // change made
    
    // Debug: Print the received command
    Serial.print("Received command: ");
    Serial.println(command);
    
    Serial.flush();  // Clear the buffer to ensure no leftover data
    
    if (command == '1') {
      // Retract both actuators
      digitalWrite(input1Pin1, LOW);
      digitalWrite(input2Pin1, HIGH);
      analogWrite(enablePin1, 150);  // Example PWM value

      digitalWrite(input1Pin2, LOW);
      digitalWrite(input2Pin2, HIGH);
      analogWrite(enablePin2, 150);  // Example PWM value

      Serial.println("Retracting actuators");  // Response message
    } else if (command == '0') {
      // Extend both actuators
      digitalWrite(input1Pin1, HIGH);
      digitalWrite(input2Pin1, LOW);
      analogWrite(enablePin1, 150);

      digitalWrite(input1Pin2, HIGH);
      digitalWrite(input2Pin2, LOW);
      analogWrite(enablePin2, 150);

      Serial.println("Extending actuators");
    } else {
      Serial.println("Unknown command");
    }
  }
}
