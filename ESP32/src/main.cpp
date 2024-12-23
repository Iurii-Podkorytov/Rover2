#include <Arduino.h>
#include <BluetoothSerial.h>
#include "esp32-hal-ledc.h"
#include "FastLED.h"
#include <AsyncTimer.h>

// #define DEBUG_ENABLE // Un-comment for debug
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif

#define in(a, x, y) (a >= x && a <= y) 

// API
#define STOP 0
#define JOYSTICK 1
#define LID 2
#define LID_OPEN 1
#define LID_CLOSE 0
#define AUDIO 3

AsyncTimer timer;
BluetoothSerial SerialBT;

// PWM setting
const int frequency = 30000;   
const int pwmChannelL1 = 0;
const int pwmChannelL2 = 1;
const int pwmChannelR1 = 2;
const int pwmChannelR2 = 3;
const int pwmChannelLid1 = 4;
const int pwmChannelLid2 = 5;

const int resolution = 8;

const int max_speed = pow(2, resolution)-1;

#define MOTOR_LID1 13 // D0
#define MOTOR_LID2 27 // D1
#define MOTOR_R1 26 // D0
#define MOTOR_R2 25 // D1
#define MOTOR_L1 33 // D2
#define MOTOR_L2 32 // D3

struct motor {
    uint8_t in1;
    uint8_t in2;
    uint8_t channel1;
    uint8_t channel2;
    int speed;
    bool reversed;
};

motor motorL = {MOTOR_L1, MOTOR_L2, pwmChannelL1, pwmChannelL2, 0, true};
motor motorR = {MOTOR_R1, MOTOR_R2, pwmChannelR1, pwmChannelR2, 0, true};
motor motorLid = {MOTOR_LID1, MOTOR_LID2, pwmChannelLid1, pwmChannelLid2, 0, false};

const int min_duty = 80;
const int max_acc = 50;

#define LEDS_PIN 14

const uint8_t num_leds = 24;
CRGB leds[num_leds];

void setup_motor(motor motor);
void set_speed(motor*, int);
void tick_L();
void tick_R();
void stop();
void drive(byte, byte);

void open_lid();
void close_lid();

void fill(CRGB);
void fade_right();
void fade_left();

void setup() {
  #ifdef DEBUG_ENABLE
    Serial.begin(115200);
    SerialBT.begin("Rover_DEBUG");
  #else
  SerialBT.begin("Rover");
  #endif
  DEBUG("Ready");

  FastLED.addLeds<WS2811, LEDS_PIN, GRB>(leds, num_leds).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(127);

  setup_motor(motorL);
  setup_motor(motorR);
  setup_motor(motorLid);

  fill(CRGB(255, 0, 0));
}


void loop() {
  timer.handle();
  static uint8_t buffer[3] = { 0 };

  // Check for incoming serial data
  if (SerialBT.available() >= 3) {
    SerialBT.readBytes(buffer, 3);
    switch (buffer[0])
    {
    case STOP:
      DEBUG("Stop");
      DEBUG(motorL.speed);
      DEBUG(motorR.speed);
      stop();
      break;
    case JOYSTICK:
      #ifdef DEBUG_ENABLE
        char string[16];
        sprintf(string, "J\t X %d Y %d", buffer[1], buffer[2]);
        DEBUG(string);
      #endif
      drive(buffer[1], buffer[2]);
      break;
    case LID:
      DEBUG("Lid");
      if (buffer[1] == LID_OPEN)
          open_lid();
      else 
          close_lid();
      break;
    case AUDIO:
      break;
    default:
      DEBUG("None");
      stop();
      break;
    }
  }
}

void setup_motor(motor motor) {
  ledcSetup(motor.channel1, frequency, resolution);
  ledcSetup(motor.channel2, frequency, resolution);
  pinMode(motor.in1, OUTPUT);
  pinMode(motor.in2, OUTPUT);
  ledcAttachPin(motor.in1, motor.channel1);
  ledcAttachPin(motor.in2, motor.channel2);
}

int dutyR = 0;
int dutyL = 0;
unsigned short idR = 0;
unsigned short idL = 0;

void drive(byte X, byte Y) {
  int forward = map(Y, 0, 254, -max_speed, max_speed);
  int turn = -map(X, 0, 254, -max_speed, max_speed);

  float angle = atan2(forward, turn);
  if(angle < 0) angle += 2*PI;
  int radius = (int) constrain(sqrt(turn * turn +  forward * forward)/2, 0, 127);

  #ifdef DEBUG_ENABLE
    char string[32];
    sprintf(string, "P\t A %f R %d", angle, radius);
    DEBUG(string);
  #endif

  int _dutyR;
  int _dutyL;
  float _angle = angle*6;
  if((in(_angle, 0, PI) || in(_angle, 11*PI, 12*PI))) { // left turn
    if(radius > 60) {
      _dutyL = -max_speed; 
      _dutyR = max_speed; 
    } else {
      _dutyL = 0; 
      _dutyR = 0; 
    }
    // _dutyL = -floor(max_speed * radius / 127);
    // _dutyR = floor(max_speed * radius / 127); 
    fade_left();
  }
  else if (in(_angle, PI, 2*PI)) { // forward-left
    _dutyL = min_duty;
    _dutyR = floor(max_speed * radius / 127); 
  }
  else if (in(_angle, 2*PI, 4*PI)) { // forward
    _dutyL = floor(max_speed * radius / 127);
    _dutyR = floor(max_speed * radius / 127); 
  }
  else if (in(_angle, 4*PI, 5*PI)) { // forward-right
    _dutyL = floor(max_speed * radius / 127); 
    _dutyR = min_duty;
  }
  else if (in(_angle, 5*PI, 7*PI)) { // right turn
    if(radius > 60) {
      _dutyL = max_speed; 
      _dutyR = -max_speed; 
    } else {
      _dutyL = 0; 
      _dutyR = 0; 
    }
    // _dutyL = floor(max_speed * radius / 127);
    // _dutyR = -floor(max_speed * radius / 127); 
    fade_right();
  } 
  else if (in(_angle, 7*PI, 8*PI)) { // back-right
    _dutyL = -floor(max_speed * radius / 127); 
    _dutyR = -min_duty;
  }
  else if (in(_angle, 8*PI, 10*PI)) { // back
    _dutyL = -floor(max_speed * radius / 127);
    _dutyR = -floor(max_speed * radius / 127); 
  }
  else if (in(_angle, 10*PI, 11*PI)) { // back-left
    _dutyL = -min_duty;
    _dutyR = -floor(max_speed * radius / 127); 
  }

  #ifdef DEBUG_ENABLE
    char message[32];
    sprintf(message, "D\t L %d R %d", _dutyL, _dutyR);
    DEBUG(message);
  #endif

  if (_dutyL != dutyL) {
    dutyL = _dutyL;
    if (idL != 0) {
      DEBUG("New L");
      timer.cancel(idL);
      idL = 0;
    }
    idL = timer.setInterval(tick_L, 50);
  }

  if (_dutyR != dutyR) {
    dutyR = _dutyR;
    if (idR != 0) {
      DEBUG("New R");
      timer.cancel(idR);
      idR = 0;
    }
    idR = timer.setInterval(tick_R, 50);
  }
}

void tick_L() {
  int speed;
  int diff = dutyL - motorL.speed;
  if (diff == 0){
    timer.cancel(idL);
    idL = 0;
    DEBUG("Canceled idL");
    return;
  }

  if (diff > 0) {
    speed = constrain(motorL.speed + max_acc, -max_speed, dutyL);
  }
  else if (diff < 0){
    speed = constrain(motorL.speed - max_acc, dutyL, max_speed);
  }
  DEBUG("ticked L");
  DEBUG(speed);
  DEBUG(motorL.speed);
  set_speed(&motorL, speed);
}

void tick_R() {
  int speed;
  int diff = dutyR - motorR.speed;
  if (diff == 0){
    timer.cancel(idR);
    idR = 0;
    DEBUG("Canceled idR");
    return;
  }

  if (diff > 0) {
    speed = constrain(motorR.speed + max_acc, -max_speed, dutyR);
  }
  else if (diff < 0){
    speed = constrain(motorR.speed - max_acc, dutyR, max_speed);
  }
  DEBUG("ticked R");
  DEBUG(speed);
  DEBUG(motorR.speed);
  set_speed(&motorR, speed);
}

void stop() {
  // set_speed(motorL, 0);
  // set_speed(motorR, 0);
  drive(127, 127);
  fill(CRGB(255, 0, 0));
}


void open_lid() {
  set_speed(&motorLid, max_speed);
}

void close_lid() {
  set_speed(&motorLid, -max_speed);
}

void set_speed(motor *motor, int speed) {
  motor->speed = speed;
  if (motor->reversed) speed = -speed;
  if (speed > 10){
    if (speed < min_duty) speed = min_duty;
    ledcWrite(motor->channel1, speed);
    ledcWrite(motor->channel2, LOW);
  }
  else if (speed < -10) {
    if (speed > -min_duty) speed = -min_duty;
    ledcWrite(motor->channel2, -speed);
    ledcWrite(motor->channel1, LOW);
  } else {
    ledcWrite(motor->channel2, LOW);
    ledcWrite(motor->channel1, LOW);
  }
  #ifdef DEBUG_ENABLE
    char string[16];
    char m;
    if(motor->in1 == motorL.in1) m = 'L';
    else m = 'R';
    sprintf(string, "Speed\t %c %d", m, speed);
    DEBUG(string);
  #endif
}

void fade_right() {
  fill(CRGB(0, 0, 0));
  for (byte i = 0; i < num_leds / 2; i++)
  {
    leds[i] = CRGB(round(255 * 2 * (num_leds/2-i)/num_leds), 0, 0); 
  }
  FastLED.show();
}

void fade_left() {
  fill(CRGB(0, 0, 0));
  for (uint8_t i = 0; i < num_leds / 2; i++)
  {
    leds[num_leds/2 + i] = CRGB(round(255 * 2 * i/num_leds), 0, 0); 
  }
  FastLED.show();
}

void fill(CRGB color) {
  for (uint8_t i = 0; i < num_leds; i++)
  {
    leds[i] = color;
  }
  FastLED.show();
}