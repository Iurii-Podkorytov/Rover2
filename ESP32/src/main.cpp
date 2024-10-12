#include <Arduino.h>
#include <BluetoothSerial.h>
#include "GyverMotor.h"

BluetoothSerial SerialBT;
int BTData;

GMotor motorL(DRIVER2WIRE, 17, 16, HIGH);
GMotor motorR(DRIVER2WIRE, 19, 18, HIGH);

#define JOYSTICK 1

void drive();

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Rover");
  Serial.println("Bluetooth Started! Ready to pair...");

  // Initialize motors
  motorL.setMode(FORWARD);
  motorR.setMode(FORWARD);
}

void loop() {
  static uint8_t buffer[8] = { 0 };
  static int index = 0;

  // Check for incoming serial data
  if (SerialBT.available()) {
    uint8_t byte = SerialBT.read();
    // Add received byte to buffer
    buffer[index++] = byte;

    // If we've received 8 bytes, parse them
    if (index == 8) {
      // Parse the received bytes here
      uint8_t cmd = buffer[0];

      switch (cmd) {
        case JOYSTICK:
          uint8_t X = buffer[1];
          uint8_t Y = buffer[2];

          // Serial.print(X);
          // Serial.print(' ');
          // Serial.println(Y);
          drive(X, Y);
          break;
      }

      memset(buffer, 0, sizeof(buffer));
      index = 0;
    }
  }
}

void drive(byte joystickX, byte joystickY) {
  // Define the range for speed (0 to 255)
  const int maxSpeed = 255;
  const int center = 125;

  // Calculate the deviation from the center position
  int forward = joystickX - center;  // Forward if positive, backward if negative
  int turn = joystickY - center;     // Right if positive, left if negative

  // Calculate the speed for each motor
  int speedL = constrain(forward - turn, -maxSpeed, maxSpeed);
  int speedR = constrain(forward + turn, -maxSpeed, maxSpeed);

  // Map the speeds to the motor drive range (0 to 255)
  int motorSpeedL = map(speedL, -maxSpeed, maxSpeed, 0, 255);
  int motorSpeedR = map(speedR, -maxSpeed, maxSpeed, 0, 255);

  // Drive the motors
  motorL.smoothTick(speedL);
  motorR.smoothTick(speedR);
}
