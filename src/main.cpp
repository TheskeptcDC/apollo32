#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

// Define motors
AF_DCMotor rightmotor(1);
AF_DCMotor leftmotor(2);

// Define the pins for SoftwareSerial (choose pins that suit your board)
#define ESP_RX 2  // ESP32 TX to Arduino RX
#define ESP_TX 3  // ESP32 RX to Arduino TX

// Initialize SoftwareSerial
SoftwareSerial espSerial(ESP_RX, ESP_TX);

// Function to map percentage to motor speed
int speed(int percent) {
  return map(percent, 0, 100, 0, 255);
}

char command = ' ';  // Variable to store the Bluetooth command

void setup() {
  // Start SoftwareSerial for ESP32 communication
  espSerial.begin(9600);
  
  // Start hardware Serial for USB communication
  Serial.begin(9600);  
  Serial.println("ESP Control Ready");
}

void loop() {
  // Check if a command is available from ESP32
  if (espSerial.available()) {
    command = espSerial.read();  // Read the command from ESP32
    Serial.print("Command from ESP32: ");
    Serial.println(command);

    // Execute the command
    switch (command) {
      case 'Right':  // Forward
        Serial.println("Executing Forward Command");
        rightmotor.setSpeed(speed(100));
        rightmotor.run(FORWARD);  // Right motor moves forward
        leftmotor.setSpeed(speed(100));
        leftmotor.run(FORWARD);  // Left motor moves forward
        break;
      case 'B':  // Backward
        Serial.println("Executing Backward Command");
        rightmotor.setSpeed(speed(100));
        rightmotor.run(BACKWARD);  // Right motor moves backward
        leftmotor.setSpeed(speed(100));
        leftmotor.run(BACKWARD);  // Left motor moves backward
        break;
      case 'S':  // Stop
        Serial.println("Executing Stop Command");
        rightmotor.run(RELEASE);  // Stop right motor
        leftmotor.run(RELEASE);  // Stop left motor
        break;
      // Add other commands here
      default:
        Serial.println("Unknown command");
        break;
    }
  }
}
