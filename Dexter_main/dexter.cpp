#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Adafruit_VL6180X.h>

#define TRUE 1
#define TIME 10
#define FREQ 1000
#define RES 8
#define PIN_MOTOR_A1 9
#define PIN_MOTOR_B1 16
#define PIN_MOTOR_B2 17
#define PIN_MOTOR_A2 18 

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

#define SHT_LOX1 7
#define SHT_LOX2 6
#define SHT_LOX3 5

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define R180 180
#define R90  90
#define GZN  131.0
#define DTN  1000.0 

#define SL = 12; 

int sL [2];

MPU6050 sensor;

float giros [2];
int distance[3];
int16_t located [5];

Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();

void start(){
  Serial.begin(115200);
  Wire1.begin(SDA, SCL);
}

void setID() {
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(TIME);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(TIME);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  if (!lox1.begin(&Wire1)) while (TRUE);
  lox1.setAddress(LOX1_ADDRESS);
  delay(TIME);

  digitalWrite(SHT_LOX2, HIGH);
  delay(TIME);

  if (!lox2.begin(&Wire1)) while (TRUE);
  lox2.setAddress(LOX2_ADDRESS);

  digitalWrite(SHT_LOX3, HIGH);
  delay(TIME);

  if (!lox3.begin(&Wire1)) while (TRUE);
  lox3.setAddress(LOX3_ADDRESS);
}

void start_sensors(){
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  setID();
  ledcAttachPin(PIN_MOTOR_A1, CHANNEL1);
  ledcSetup(CHANNEL1, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_A2, CHANNEL2);
  ledcSetup(CHANNEL2, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_B1, CHANNEL3);
  ledcSetup(CHANNEL3, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_B2, CHANNEL4);
  ledcSetup(CHANNEL4, FREQ, RES);
}

void pinModes(){
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
}

void moveForward() {
  ledcWrite(CHANNEL1, HIGH);  
  ledcWrite(CHANNEL2, LOW);
  ledcWrite(CHANNEL3, HIGH);  
  ledcWrite(CHANNEL4, LOW);
}

void moveRight() {
  ledcWrite(CHANNEL1, HIGH); 
  ledcWrite(CHANNEL2, LOW);
  ledcWrite(CHANNEL3, LOW);   
  ledcWrite(CHANNEL4, HIGH);
}

void moveLeft() {
  ledcWrite(CHANNEL1, LOW);   
  ledcWrite(CHANNEL2, HIGH);
  ledcWrite(CHANNEL3, HIGH);  
  ledcWrite(CHANNEL4, LOW);
}

void stopMotors() {
  ledcWrite(CHANNEL1, LOW);
  ledcWrite(CHANNEL2, LOW);
  ledcWrite(CHANNEL3, LOW);
  ledcWrite(CHANNEL4, LOW);
}

void ml(int *msl){  //sensor piso
  *msl = digitalRead(SL);  
  if (*msl== HIGH && *msl + 1  == 0) {  
    if (analogRead(SL) < 512) {  
      stopMotors(); 
    } else {  
    int state = 0;   
    }
    *msl + 1 = 1;  
  } else if (*msl== LOW) {
    *msl = 0;  
  }
}

void measurement(int *dr){ //medir distancia sensores
  uint8_t range;
  range = lox1.readRange();
  if (lox1.readRangeStatus() == VL6180X_ERROR_NONE) *(dr) = range;
  else *(dr) = -1;

  range = lox2.readRange();
  if (lox2.readRangeStatus() == VL6180X_ERROR_NONE) *(dr+1) = range;
  else *(dr+1) = -1;

  range = lox3.readRange();
  if (lox3.readRangeStatus() == VL6180X_ERROR_NONE) *(dr+2) = range;
  else *(dr+2) = -1;

}

 void rotate(int *L, float *G){ //para que rote el mpu6050
    float initialAngle = *(G);
    while (*(G) - initialAngle < R90) {
    sensor.getRotation(L, L+1, L+2);
    *(L+5) = millis() - *(L+4);
    *(L+4)= millis();
    *(G) = (*(L+2) / GZN) * *(L+5 )/ DTN + *(G+1);
    *(G+1) = *(G);
    }
    delay(TIME);
}

void rotateL90() { 
    rotate();
    moveLeft();
    stopMotors();
}

void rotateR90() {
  rotate()
  moveRight();
  stopMotors();
}

void movements(){ //los movimientos dependiendo el caso
  uint8_t state = 0;
  switch (state) {
    case 1: // No walls (000)
      ml ();
      break;
    case 2: // Wall to the right (001)
      moveForward();
      break;
    case 3: // Wall in front (010)
      rotateR90();
      break;
    case 4: // Wall in front and right (011)
      rotateL90();
      break;
    case 5: // Wall to the left (100)
      rotateL90();
      break;
    case 6: // Pared a la izquierda y derecha (101)
      moveForward();
      break;
    case 7: // Pared al frente y a la izquierda (110)
      rotateR90();
      break;
    case 8: // Pared en todas las direcciones (111)
      rotateL90();
      break;
  }
}

void setup() {
  pinModes();
  setID();
}

void loop() {

}
