#include <Arduino.h>
#include <BluetoothSerial.h>
#include "esp32-hal-ledc.h"

#define STOP 0
#define JOYSTICK 1
#define LID_OPEN 2
#define LID_CLOSE 3
#define AUDIO 4

BluetoothSerial SerialBT;

// PWM setting
const int frequency = 30000;   
const int pwmChannelL = 0;
const int pwmChannelR = 1;
const int resolution = 8;

#define MOTOR_L_PWM 16 // D0
#define MOTOR_L_DIR 17 // D1
#define MOTOR_R_PWM 18 // D2
#define MOTOR_R_DIR 19 // D3
// #define MOTOR_LID_PWM 10
// #define MOTOR_LID_DIR 11

struct motor {
    uint8_t in1;
    uint8_t in2;
};

motor motorL = {MOTOR_L_PWM, MOTOR_L_DIR};
motor motorR = {MOTOR_R_PWM, MOTOR_R_DIR};
// motor motorLid = {MOTOR_LID_PWM, MOTOR_LID_DIR};

void drive(byte, byte);
void set_speed(motor, int);
// void open_lid();
// void close_lid();
// void play_audio(byte);
void stop();

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Rover");
  Serial.println("Bluetooth Started! Ready to pair...");

  pinMode(motorL.in1, OUTPUT);
  pinMode(motorR.in1, OUTPUT);
  // pinMode(motorLid.dirPin, OUTPUT);
  pinMode(motorL.in2, OUTPUT);
  pinMode(motorR.in2, OUTPUT);
  // pinMode(motorLid.pwmPin, OUTPUT);

  ledcSetup(pwmChannelL, frequency, resolution);
  ledcSetup(pwmChannelR, frequency, resolution);
  
  ledcAttachPin(motorL.in1, pwmChannelL);
  ledcAttachPin(motorL.in2, pwmChannelL);

  ledcAttachPin(motorR.in1, pwmChannelR);
  ledcAttachPin(motorR.in2, pwmChannelR);
}


void loop() {
  static uint8_t buffer[8] = { 0 };

  // Check for incoming serial data
  if (SerialBT.available() >= 8) {
    SerialBT.readBytes(buffer, 8);
    switch (buffer[0])
    {
    case STOP:
      stop();
      break;
    case JOYSTICK:
      Serial.print("X ");
      Serial.print(buffer[1]);
      Serial.print(" Y ");
      Serial.println(buffer[2]);
      drive(buffer[1], buffer[2]);
      break;
    case LID_OPEN:
      // open_lid();
      break;
    case LID_CLOSE:
      // close_lid();
      break;
    case AUDIO:
      // play_audio(buffer[1]);
      break;
    default:
      stop();
      break;
    }
  }
}

void stop() {
  set_speed(motorL, 0);
  set_speed(motorR, 0);
}

void drive(byte X, byte Y) {
  const int max_speed = pow(2, resolution)-1;

  int forward = map(Y, 0, 250, -max_speed, max_speed);
  int turn = -map(X, 0, 250, -max_speed, max_speed);

  int dutyR = constrain(turn+forward, -max_speed, max_speed);
  int dutyL = constrain(turn-forward, -max_speed, max_speed);

  set_speed(motorL, dutyL);
  set_speed(motorR, dutyR);

  Serial.print("L ");
  Serial.print(dutyL);
  Serial.print(" R ");
  Serial.println(dutyR);

}

void set_speed(motor motor, int speed) {
  // if (speed >= 0){
  //   ledcWrite(motor.in1, speed);
  //   ledcWrite(motor.in2, LOW);
  // }
  // else {
  //   ledcWrite(motor.in2, -speed);
  //   ledcWrite(motor.in1, LOW);
  // } 
  if (speed >= 0){
    analogWrite(motor.in1, speed);
    analogWrite(motor.in2, LOW);
  }
  else {
    analogWrite(motor.in2, -speed);
    analogWrite(motor.in1, LOW);
  } 
}