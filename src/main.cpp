#include <AFMotor.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

// Define motors
AF_DCMotor rightmotor(1);
AF_DCMotor leftmotor(2);

// Define the pins for SoftwareSerial (choose pins that suit the board)
#define ESP_RX 0  // ESP32 TX to Arduino RX
#define ESP_TX 1  // ESP32 RX to Arduino TX

// Initialize SoftwareSerial
SoftwareSerial espSerial(ESP_RX, ESP_TX);

// Function to map percentage to motor speed
int speed(int percent) {
  return map(percent, 0, 100, 0, 255);
}

// SPEED 
int acceleration = 0;

char command = ' ';  // Variable to store the Bluetooth command

Servo myservo;  // create servo object to control a servo
int servo1 = 9;  // first servo motor 
int servo2 = 10; // second servo motor

void setup() {
  // Start SoftwareSerial for ESP32 communication
  espSerial.begin(9600);
  
  // Start hardware Serial for USB communication
  Serial.begin(9600);  
  Serial.println("ESP Control Ready");

  // Attach the servo to the pin
  myservo.attach(servo1);
  // myservo.attach(servo2);
}

void loop() {
  // Check if a command is available from ESP32
  if (espSerial.available()) {
    command = espSerial.read();  // Read the command from ESP32
    Serial.print("Command from ESP32: ");
    Serial.println(command);
    if (command == 'q')
    {
      acceleration = 100;
    }
    else if (command == 3)
    {
      acceleration = 50;
    }
    else
    {
      acceleration = 20;
    }
    
    // Execute the command
    switch (command) {
      case 'F':  // Forward
        Serial.println("Executing Forward Command");
        rightmotor.setSpeed(speed(acceleration));
        rightmotor.run(FORWARD);  // Right motor moves forward
        leftmotor.setSpeed(speed(100));
        leftmotor.run(FORWARD);  // Left motor moves forward
        break;
      case 'B':  // Backward
        Serial.println("Executing Backward Command");
        rightmotor.setSpeed(speed(acceleration));
        rightmotor.run(BACKWARD);  // Right motor moves backward
        leftmotor.setSpeed(speed(100));
        leftmotor.run(BACKWARD);  // Left motor moves backward
        break;
      case 'S':  // Stop
        Serial.println("Executing Stop Command");
        rightmotor.run(RELEASE);  // Stop right motor
        leftmotor.run(RELEASE);  // Stop left motor
        break;
      case 'R':  // RIGHT
        Serial.println("Executing RIGHT Command");
        rightmotor.run(BACKWARD);  // right motor
        leftmotor.run(FORWARD);  // left motor
        break;
      case 'L':  // LEFT
        Serial.println("Executing LEFT Command");
        rightmotor.run(FORWARD);  // Stop right motor
        leftmotor.run(BACKWARD);  // Stop left motor
        break;
      case 'U':  // Open servo
        Serial.println("Opening arm");
        myservo.write(180);  //
        break;
      case 'u':  // Close servo
        Serial.println("Closing arm");
        myservo.write(0);  // close serv
        break;
      default:
        Serial.println("Unknown command");
        break;
    }
  }
}
