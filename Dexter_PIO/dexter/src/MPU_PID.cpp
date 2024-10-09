#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_VL6180X.h>

// Parámetros del PID
#define KP 5.0          // Ganancia proporcional
#define KD 0.1          // Ganancia derivativa
#define BASE_SPEED 127  // Velocidad base de los motores (0 - 255)
#define FREQ 1000
#define RES 8
#define MAX 100
#define MIN 0

#define PIN_MOTOR_A1 26
#define PIN_MOTOR_A2 27
#define PIN_MOTOR_B1 13
#define PIN_MOTOR_B2 14
#define PIN_BOTON_START 18

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

#define SHT_LOX1 7
#define SHT_LOX2 6
#define SHT_LOX3 5

#define GZN  131.0
#define DTN  1000.0
#define SGM 2
#define DM 3
#define LM 5
#define SL 12 // Sensor de piso

#define GIROS_ANG_X 0
#define GIROS_ANG_X_PREV 1
#define DISTANCE1 0 
#define DISTANCE2 1
#define DISTANCE3 2
#define GX 0
#define GY 1
#define GZ 2
#define TIME_PREV 3
#define DT 4
MPU6050 sensor;

float giros[SGM] = { GIROS_ANG_X , GIROS_ANG_X_PREV };  // Arreglo para giroscopio (girosc_ang_x, girosc_ang_x_prev)
int16_t distance[DM] = {DISTANCE1, DISTANCE2,DISTANCE3};  // Arreglo para las distancias de los sensores (distance1, distance2,distance3)
int16_t located[LM] = {GX, GY, GZ, TIME_PREV, DT };  // Arreglo para valores del MPU6050 (gx, gy, gz, tiempo_prev, dt)

float* G = giros;
int16_t* L = located;
int16_t* dr = distance;

Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();

bool boton_presionado = false;
float previousError = 0;

// Función de reposo
void reposo() {
  while (!boton_presionado) {
    if (digitalRead(PIN_BOTON_START) == LOW) {
      boton_presionado = true;
      delay(200);  // Anti-rebote
    }
  }
}

// Configurar los ID de los sensores
void setID() {
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  delay(10);
  if (!lox1.begin(&Wire1)) Serial.println("Error iniciando lox1");
  lox1.setAddress(LOX1_ADDRESS);

  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  if (!lox2.begin(&Wire1)) Serial.println("Error iniciando lox2");
  lox2.setAddress(LOX2_ADDRESS);

  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  if (!lox3.begin(&Wire1)) Serial.println("Error iniciando lox3");
  lox3.setAddress(LOX3_ADDRESS);
}

void start_sensors() {
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  setID();
  
  // Configuración de los pines de los motores
  ledcAttachPin(PIN_MOTOR_A1, CHANNEL1);
  ledcSetup(CHANNEL1, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_A2, CHANNEL2);
  ledcSetup(CHANNEL2, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_B1, CHANNEL3);
  ledcSetup(CHANNEL3, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_B2, CHANNEL4);
  ledcSetup(CHANNEL4, FREQ, RES);
}

void readDistance(Adafruit_VL6180X &vl, uint16_t &distance) {
  uint16_t range = vl.readRange();
  distance = range;
}

void measurement(int16_t *dr) {
  uint16_t distance1, distance2, distance3;
  readDistance(lox1, distance1);
  *dr = distance1;
  readDistance(lox2, distance2);
  *(dr + 1) = distance2;
  readDistance(lox3, distance3);
  *(dr + 2) = distance3;
}

void moveForward() {
  ledcWrite(CHANNEL1, MAX);  
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MAX);  
  ledcWrite(CHANNEL4, MIN);
}

void stopMotors() {
  ledcWrite(CHANNEL1, MIN);
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MIN);
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

// PID para el sensor VL6180X
void pidControl() {
  float targetDistance = 75;  // Distancia objetivo
  float error, derivative, pidOutput;
  
  readDistance(lox3, distance[2]);
  Serial.print("Distancia: ");
  Serial.println(distance[2]);

  error = targetDistance - distance[2];
  derivative = error - previousError;
  pidOutput = (KP * error) + (KD * derivative);
  previousError = error;

  int motorSpeedLeft = constrain(BASE_SPEED + pidOutput, MIN, MAX);
  int motorSpeedRight = constrain(BASE_SPEED - pidOutput, MIN, MAX);

  controlMotor(motorSpeedLeft, motorSpeedRight);
  delay(100);
}

void controlMotor(int speedLeft, int speedRight) {
  if (speedLeft > 0) {
    ledcWrite(CHANNEL1, speedLeft);  
    ledcWrite(CHANNEL2, MIN);       
  } else {
    ledcWrite(CHANNEL1, MIN);       
    ledcWrite(CHANNEL2, abs(speedLeft));  
  }

  if (speedRight > 0) {
    ledcWrite(CHANNEL3, speedRight); 
    ledcWrite(CHANNEL4, MIN);       
  } else {
    ledcWrite(CHANNEL3, MIN);       
    ledcWrite(CHANNEL4, abs(speedRight));  
  }
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
    int8_t state = 0;  
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
            pidControl()
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
  Wire.begin();
  Serial.begin(115200);
  pinMode(PIN_BOTON_START, INPUT_PULLUP);
  start_sensors();
}

void loop() {
  reposo();
  pidControl();
  movements();
}
