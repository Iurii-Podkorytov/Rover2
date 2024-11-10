#include <Arduino.h>
#include <BluetoothSerial.h>
#include "esp32-hal-ledc.h"
#include "FastLED.h"
// #include <AsyncTimer.h>

#define in(a, x, y) (a >= x && a < y) 

#define STOP 0
#define JOYSTICK 1
#define LID 2
#define LID_OPEN 1
#define LID_CLOSE 0
#define AUDIO 3

// AsyncTimer timer;
BluetoothSerial SerialBT;

// PWM setting
const int frequency = 30000;   
const int pwmChannelL1 = 0;
const int pwmChannelL2 = 1;
const int pwmChannelR1 = 2;
const int pwmChannelR2 = 3;
const int pwmChannelLID1 = 4;
const int pwmChannelLID2 = 5;

const int resolution = 8;

const int max_speed = pow(2, resolution)-1;

#define MOTOR_LID1 13 // D0
#define MOTOR_LID2 27 // D1
#define MOTOR_R1 26 // D0
#define MOTOR_R2 25 // D1
#define MOTOR_L1 33 // D2
#define MOTOR_L2 32 // D3

#define LEDS_PIN 14 // HIGH on start up. Leds will blink white

const uint8_t num_leds = 24;
CRGB leds[num_leds];
uint8_t anim_id;
struct motor {
    uint8_t in1;
    uint8_t in2;
    uint8_t channel1;
    uint8_t channel2;
    int speed;
};

motor motorL = {MOTOR_L1, MOTOR_L2, pwmChannelL1, pwmChannelL2, 0};
motor motorR = {MOTOR_R1, MOTOR_R2, pwmChannelR1, pwmChannelR2, 0};
motor motorLid = {MOTOR_LID1, MOTOR_LID2, pwmChannelLID1, pwmChannelLID2, 0};

const int min_duty = 80;

void drive(byte, byte);
void set_speed(motor, int);
void open_lid();
void close_lid();
void brake();
// void play_audio(byte);
void stop();
void fade_right();
void fade_left();
void fill();
void leds_off();

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Rover");
  Serial.println("Bluetooth Started! Ready to pair...");

  FastLED.addLeds<WS2811, LEDS_PIN, GRB>(leds, num_leds).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(127);
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
  ledcSetup(pwmChannelLID1, frequency, resolution);
  ledcSetup(pwmChannelLID2, frequency, resolution);
  
  ledcAttachPin(motorL.in1, pwmChannelL1);
  ledcAttachPin(motorL.in2, pwmChannelL2);

  ledcAttachPin(motorR.in1, pwmChannelR1);
  ledcAttachPin(motorR.in2, pwmChannelR2);

  ledcAttachPin(motorLid.in1, pwmChannelLID1);
  ledcAttachPin(motorLid.in2, pwmChannelLID2);

  fill();
}


void loop() {
  static uint8_t buffer[3] = { 0 };

  // Check for incoming serial data
  if (SerialBT.available() >= 3) {
    SerialBT.readBytes(buffer, 3);

    switch (buffer[0])
    {
    case STOP:
      stop();
      Serial.println('S');
      break;
    case JOYSTICK:
      // Serial.println('J');
      Serial.print("X ");
      Serial.print(buffer[1]);
      Serial.print(" Y ");
      Serial.println(buffer[2]);
      drive(buffer[1], buffer[2]);
      break;
    case LID:
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
      Serial.println('D');
      stop();
      break;
    }
  }
}

void stop() {
  set_speed(motorL, 0);
  set_speed(motorR, 0);
  fill();
}

void drive(byte X, byte Y) {
  int forward = map(Y, 0, 254, -max_speed, max_speed);
  int turn = -map(X, 0, 254, -max_speed, max_speed);
  float angle = atan2(forward, turn);
  if(angle < 0) angle += 2*PI;
  int radius = (int) constrain(sqrt(turn * turn +  forward * forward)/2, 0, 127);

  Serial.print("A ");
  Serial.print(angle);
  Serial.print(" R ");
  Serial.println(radius);

  int dutyR = constrain(forward+turn, -max_speed, max_speed);
  int dutyL = constrain(forward-turn, -max_speed, max_speed);

  float _angle = angle*6;
  if (in(_angle, 5*PI, 7*PI) && radius > 60) {
    dutyR = max_speed;
    dutyL = -max_speed;
    fade_right();
  } 
  else if((in(_angle, 0, PI) || in(_angle, 11*PI, 12*PI)) && radius > 60) {
    dutyR = -max_speed;
    dutyL = max_speed;
    fade_left();
  }

  Serial.print("L ");
  Serial.print(dutyL);
  Serial.print(" R ");
  Serial.println(dutyR);

  // setSpeed(motorL, dutyL);
  // setSpeed(motorR, dutyR);
  set_speed(motorL, dutyL); 
  set_speed(motorR, -dutyR); // right side connection is reversed
}

void open_lid() {
      Serial.println('O');
  set_speed(motorLid, max_speed);
}

void close_lid() {
      Serial.println('C');
  set_speed(motorLid, -max_speed);
}

void brake() {
  set_speed(motorL, max_speed);
  set_speed(motorR, max_speed);
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

void leds_off() {
  for (uint8_t i = 0; i < num_leds; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}

void fade_right() {
  leds_off();
  for (byte i = 0; i < num_leds / 2; i++)
  {
    leds[i] = CRGB(round(255 * 2 * (num_leds/2-i)/num_leds), 0, 0); 
  }
  FastLED.show();
}

void fade_left() {
  leds_off();
  for (uint8_t i = 0; i < num_leds / 2; i++)
  {
    leds[num_leds/2 + i] = CRGB(round(255 * 2 * i/num_leds), 0, 0); 
  }
  FastLED.show();
}

void fill() {
  for (uint8_t i = 0; i < num_leds; i++)
  {
    leds[i] = CRGB(255, 0, 0);
  }
  FastLED.show();
}