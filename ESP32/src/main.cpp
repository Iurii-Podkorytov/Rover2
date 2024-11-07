#include <Arduino.h>
#include <BluetoothSerial.h>
#include "esp32-hal-ledc.h"

#define in(a, x, y) (a >= x && a < y) 

#define STOP 0
#define JOYSTICK 1
#define LID 2
#define LID_OPEN 1
#define LID_CLOSE 0
#define AUDIO 3

BluetoothSerial SerialBT;

// PWM setting
const int frequency = 30000;   
const int pwmChannelL1 = 0;
const int pwmChannelL2 = 1;
const int pwmChannelR1 = 2;
const int pwmChannelR2 = 3;
const int resolution = 8;

const int max_speed = pow(2, resolution)-1;

#define MOTOR_L1 25 // D0
#define MOTOR_L2 26 // D1
#define MOTOR_R1 27 // D2
#define MOTOR_R2 33 // D3
#define MOTOR_LID1 32 // D0
#define MOTOR_LID2 14 // D1

struct motor {
    uint8_t in1;
    uint8_t in2;
    uint8_t channel1;
    uint8_t channel2;
};

motor motorL = {MOTOR_L1, MOTOR_L2, pwmChannelL1, pwmChannelL2};
motor motorR = {MOTOR_R1, MOTOR_R2, pwmChannelR1, pwmChannelR2};
motor motorLid = {MOTOR_LID1, MOTOR_LID2, 0, 0};

const int min_duty = 80;

void drive(byte, byte);
void set_speed(motor, int);
void open_lid();
void close_lid();
// void play_audio(byte);
void stop();

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Rover");
  Serial.println("Bluetooth Started! Ready to pair...");

  pinMode(motorL.in1, OUTPUT);
  pinMode(motorL.in2, OUTPUT);

  pinMode(motorR.in1, OUTPUT);
  pinMode(motorR.in2, OUTPUT);

  pinMode(motorLid.in1, OUTPUT);
  pinMode(motorLid.in2, OUTPUT);

  ledcSetup(pwmChannelL1, frequency, resolution);
  ledcSetup(pwmChannelL2, frequency, resolution);
  ledcSetup(pwmChannelR1, frequency, resolution);
  ledcSetup(pwmChannelR2, frequency, resolution);
  
  ledcAttachPin(motorL.in1, pwmChannelL1);
  ledcAttachPin(motorL.in2, pwmChannelL2);

  ledcAttachPin(motorR.in1, pwmChannelR1);
  ledcAttachPin(motorR.in2, pwmChannelR2);
}


void loop() {
  static uint8_t buffer[3] = { 0 };

  // Check for incoming serial data
  if (SerialBT.available() >= 3) {
    SerialBT.readBytes(buffer, 3);
    SerialBT.write(buffer, sizeof(buffer));
    for (int i = 0; i < 3; i++) {
      Serial.print(buffer[i]);
      Serial.print(' ');  
    }
    Serial.println();

    switch (buffer[0])
    {
    case STOP:
      stop();
      SerialBT.write('S');
      Serial.println('S');
      break;
    case JOYSTICK:
      SerialBT.write('J');
      Serial.println('J');
      drive(buffer[1], buffer[2]);
      break;
    case LID:
      SerialBT.write('L');
      Serial.println('L');
      if (buffer[1] == LID_OPEN)
          open_lid();
      else 
          close_lid();
      break;
    case AUDIO:
      // play_audio(buffer[1]);
      break;
    default:
      SerialBT.write('D');
      Serial.println('D');
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
  int forward = map(Y, 0, 254, -max_speed, max_speed);
  int turn = -map(X, 0, 254, -max_speed, max_speed);
  float angle = atan2(forward, turn);
  if(angle < 0) angle += 2*PI;
  float radius = sqrt(turn * turn +  forward * forward)/2;

  Serial.print("A ");
  Serial.print(angle);
  Serial.print(" R ");
  Serial.println(radius);

  int dutyR = constrain(turn+forward, -max_speed, max_speed);
  int dutyL = constrain(turn-forward, -max_speed, max_speed);

  float _angle = angle*6;
  if (in(_angle, 5*PI, 7*PI) && radius > 60) { // right turn
    dutyR = -max_speed;
    dutyL = -max_speed;
    Serial.println("RIGHT");
  } 
  else if((in(_angle, 0, PI) || in(_angle, 11*PI, 12*PI)) && radius > 60) { // left turn
    dutyR = max_speed;
    dutyL = max_speed;
    Serial.println("LEFT");
}

  set_speed(motorL, dutyL);
  set_speed(motorR, dutyR);
}

void open_lid() {
  analogWrite(motorLid.in1, HIGH);
  analogWrite(motorLid.in2, LOW);
}

void close_lid() {
  analogWrite(motorLid.in2, HIGH);
  analogWrite(motorLid.in1, LOW);
}

void stop_analog(motor motor) {
  analogWrite(motorLid.in2, LOW);
  analogWrite(motorLid.in1, LOW);
}

void set_speed(motor motor, int speed) {
  if (speed >= 0){
    if (speed < min_duty) speed = min_duty;
    ledcWrite(motor.channel1, speed);
    ledcWrite(motor.channel2, LOW);
  }
  else {
    if (speed > min_duty) speed = -min_duty;
    ledcWrite(motor.channel2, -speed);
    ledcWrite(motor.channel1, LOW);
  } 
}