#include <Arduino.h>
#include <BluetoothSerial.h>
#include "GyverMotor.h"

#define MOTOR_GPIO_IN1 16
#define MOTOR_GPIO_IN2 17
#define MOTOR_GPIO_IN3 18
#define MOTOR_GPIO_IN4 19


BluetoothSerial SerialBT;
int BTData;

GMotor motorL(DRIVER2WIRE, MOTOR_GPIO_IN2, MOTOR_GPIO_IN1, HIGH);
GMotor motorR(DRIVER2WIRE, MOTOR_GPIO_IN4, MOTOR_GPIO_IN3, HIGH);

#define JOYSTICK 1
#define LED_BUILTIN 2
#define MIN_DUTY 300

void drive(uint8_t, uint8_t);

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Rover");
  Serial.println("Bluetooth Started! Ready to pair...");

  pinMode(LED_BUILTIN, OUTPUT);


  // Initialize motors
  motorL.setMode(AUTO);
  motorR.setMode(AUTO);

  motorL.setMinDuty(MIN_DUTY);
  motorR.setMinDuty(MIN_DUTY);

  motorL.setResolution(10);
  motorR.setResolution(10);

  motorL.setSmoothSpeed(50);
  motorR.setSmoothSpeed(50);
}

void loop() {
  static uint8_t buffer[8] = { 0 };
  static int index = 0;

  // Check for incoming serial data
  if (SerialBT.available()) {
    // Serial.println(SerialBT.read());
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

          Serial.print(X);
          Serial.print(' ');
          Serial.println(Y);
          drive(X, Y);
          break;
      }

      memset(buffer, 0, sizeof(buffer));
      index = 0;
    }
  }
}

void drive(uint8_t X, uint8_t Y) {
  const int max_speed = 1023;

  int forward = map(Y, 0, 255, -max_speed, max_speed);
  int turn = -map(X, 0, 255, -max_speed, max_speed);

  int dutyR = constrain(turn+forward, -max_speed, max_speed);
  int dutyL = constrain(turn-forward, -max_speed, max_speed);

  if (dutyL < 0 && dutyR > 0)
    dutyR -= 50;
  if (dutyL > 0 && dutyR < 0)
    dutyL -= 50;

  // Serial.print("Duty: ");
  // Serial.print("L "); 
  // Serial.print(dutyL); 
  // Serial.print(" R "); 
  // Serial.println(dutyR); 


  
  // motorL.setSpeed(dutyL);
  // motorR.setSpeed(dutyR);
  motorL.smoothTick(dutyL);
  motorR.smoothTick(dutyR);
}
