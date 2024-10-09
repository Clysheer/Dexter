#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Adafruit_VL6180X.h>

#define NUM_MAX 512
#define BEGIN 115200
#define TIME 10
#define FREQ 1000
#define RES 8
#define PIN_MOTOR_A1 26
#define PIN_MOTOR_A2 27
#define PIN_MOTOR_B1 13
#define PIN_MOTOR_B2 14 
#define PIN_BOTON_START 18

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

#define giros_ang_x 1

#define R180 180
#define R90  90
#define GZN  131.0
#define DTN  1000.0 
#define SGM 2 // giros
#define DM 3 //distancias
#define LM 5  //locacion laberinto
#define SL 12 //sensor piso 

MPU6050 sensor;

float giros[SGM] = { giros_ang_x , giros_ang_x_prev};  // Arreglo para giroscopio (girosc_ang_x, girosc_ang_x_prev)
int16_t distance[DM] = {distance1, distance2,distance3};  // Arreglo para las distancias de los sensores (distance1, distance2,distance3)
int16_t located[LM] = {gx, gy, gz, tiempo_prev, dt};  // Arreglo para valores del MPU6050 (gx, gy, gz, tiempo_prev, dt)

G  = giros;
L  = located;
dr = distance;

Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();

bool boton_presionado = false;

void blocked_pwm(){
  ledcSetup(0, 5000, 8); // Canal 0, 5 kHz, 8 bits de resolución
  ledcAttachPin(PIN_MOTOR_B2, 0);
}

void reposo() {
  // Espera hasta que el botón sea presionado
  while (!boton_presionado) {
    if (digitalRead(PIN_BOTON_START) == LOW) { // Verificar si el botón está presionado (LOW)
      boton_presionado = true;
      delay(200);  // Evitar debounce con un retraso
    }
  }
}

void start() {
  Serial.begin(BEGIN);
  Wire.begin();
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
  Serial.println("inciiando lox1");
   while (!lox1.begin(&Wire1)) 
   Serial.println("lox1 no sepudo iniciar!!!");

   lox1.setAddress(LOX1_ADDRESS);
   delay(TIME);

   digitalWrite(SHT_LOX2, HIGH);
   delay(TIME);

  while  (!lox2.begin(&Wire1)) 
    Serial.println("lox2 no sepudo iniciar!!!");

   lox2.setAddress(LOX2_ADDRESS);
   delay(TIME);

   digitalWrite(SHT_LOX3, HIGH);
   delay(TIME);

   while (!lox3.begin(&Wire1)) 
    Serial.println("lox3 no sepudo iniciar!!!");

    lox3.setAddress(LOX3_ADDRESS);
}

void start_sensors() {
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

void moveForward() {
  ledcWrite(CHANNEL1, MAX);  
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MAX);  
  ledcWrite(CHANNEL4, MIN);
}

void moveRight() {
  ledcWrite(CHANNEL1, MAX); 
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MIN);   
  ledcWrite(CHANNEL4, MAX);
}

void moveLeft() {
  ledcWrite(CHANNEL1, MIN);   
  ledcWrite(CHANNEL2, MAX);
  ledcWrite(CHANNEL3, MAX);  
  ledcWrite(CHANNEL4, MIN);
}

void stopMotors() {
  ledcWrite(CHANNEL1, MIN);
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MIN);
  ledcWrite(CHANNEL4, MIN);
}

void readDistance(Adafruit_VL6180X &vl, uint16_t &distance) {
  uint16_t range = vl.readRange();
  distance = range;
 }

 void measurement(int16_t *dr) { //medir distancia sensores
  uint16_t distance1, distance2, distance3;
  readDistance(lox1, distance1);
  *dr = distance1;
  readDistance(lox2, distance2);
  *(dr + 1) = distance2;
  readDistance(lox3, distance3);
  *(dr + 2) = distance3;
}


void rotate(int *L, float *G, float angle) {  // para que rote el MPU6050
    float initialAngle = *G;
    unsigned long startTime = millis();  // Registramos el tiempo de inicio
    unsigned long elapsedTime = 0;

    while (*G - initialAngle < angle && elapsedTime < TIME) {  // Comparamos también con el tiempo transcurrido
        sensor.getRotation(L, L + 1, L + 2);
        *(L + 5) = millis() - *(L + 4);
        *(L + 4) = millis();
        *G = (*(L + 2) / GZN) * (*(L + 5) / DTN) + *(G + 1);
        *(G + 1) = *G;
        elapsedTime = millis() - startTime;  // Calculamos el tiempo transcurrido
    }
} 

void rotateL90() { 
    moveLeft();
    rotate(&giros[0], &girosc_ang_x, R90); // Rote 90 grados
    stopMotors();
}

void rotateR90() {
  moveRight();
  rotate(&giros[0], &girosc_ang_x, R90); 
  stopMotors();
}

enum estadosMovimiento
{
  PISODETECTADO;
  MOVEFORWARD;
  ROTATEL90;
  ROTATER90;
  ROTATE180;
}

void movements() { 
    int8_t piso = digitalRead(SL);
    estadoMovimiento state; 
    measurement(distance); 
    if (!piso) {  
        state = PISODETECTADO;  //detecta piso
    }
    else if (distance[0] > 85 && distance[1] > 85 && distance[2] < 85 ) {   //detectan derecha
        state =  MOVEFORWARD ;   
    }
    else if (distance[0] > 85 && distance[1] < 85 && distance[2] < 85 ) {   //detectan medio y derecha 
        state = ROTATEL90 ;   
    }
   else if (distance[0] < 85 && distance[1] > 85 && distance[2] < 85 ) {   //detectan izquierda y derecha  
        state =  MOVEFORWARD;  
    }
    else if (distance[0] < 85 && distance[1] < 85 && distance[2] > 85 ) {   //detectan medio y izuierda 
        state = ROTATER90 ;   
    }
    else if (distance[0] < 85 && distance[1] > 85 && distance[2] > 85 ) {   //detectan izquierda 
        state = ROTATER90 ;   
    }
    else if (distance[0] > 85 && distance[1] < 85 && distance[2] > 85 ) {   //detectan medio 
        state = ROTATER90 ;   
    }
    else if (distance[0] < 85 && distance[1] < 85 && distance[2] < 85) {   //detectan todos 
        state = ROTATE180 ;  
    }
    else if (distance[0] > 85 && distance[1] > 85 && distance[2] > 85 ) {   // no detecta ninguno 
        state = MOVEFORWARD ;   
    }
    switch (state) {
        case 1: 
            stopMotors(); //detecta piso
            break;
        case 2: 
            moveForward(); //detectan derecha
            break;
        case 3: 
            rotateL90(); //detectan medio y derecha 
            break;
        case 4: 
            rotateR90(); //detectan medio y izuierda
            break;
        case 5: 
            rotateL180();//detectan todos 
            break;
    }
}

void setup() {
  start();
  start_sensors();
}

void loop() {
  movements();
}
