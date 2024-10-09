#include <BluetoothSerial.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_VL6180X.h>
BluetoothSerial SerialBT;
// Parámetros del PID
#define KP 4.0          // Ganancia proporcional
#define KD 0.1          // Ganancia derivativa
#define BASE_SPEED 127  // Velocidad base de los motores (0 - 255)
#define FREQ 1000
#define RES 8
#define MAX 50
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

#define SHT_LOX1 19
#define SHT_LOX2 5
#define SHT_LOX3 2

#define BEGIN 9600
#define R180 180
#define R90  90
#define GZN  131.0
#define DTN  1000.0
#define SGM 2
#define DM 3
#define LM 5
#define SL 12 // Sensor de piso

#define DISTANCE1 0 
#define DISTANCE2 1
#define DISTANCE3 2
#define SCL 22
#define SDA 21

MPU6050 sensor;
  int16_t distance[DM] = {DISTANCE1, DISTANCE2,DISTANCE3};  // Arreglo para las distancias de los sensores (distance1, distance2,distance3)
  int16_t *dr = distance;


  // Valor RAW del giroscopio en el eje z
  int16_t gz;
  int16_t gy;
  int16_t gx;
  float gz_cal=0;
  long tiempo_prev, dt;
  float elapsedTime;
  float girosc_ang_z;
  float girosc_ang_z_prev;

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
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  Serial.println("no entro jajajaja");
 if (!lox1.begin()) while (1);
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);
  Serial.println("A");
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if (!lox2.begin()) while (1);
  lox2.setAddress(LOX2_ADDRESS);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10); 
  Serial.println("B");
  if (!lox3.begin()) while (1);
  lox3.setAddress(LOX3_ADDRESS);
  Serial.println("C");
}

void start_sensors() {
  sensor.initialize();  // Inicializa el MPU6050
  pinMode(PIN_BOTON_START, INPUT_PULLUP);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  setID();
  calibrateMPU6050();
  tiempo_prev = millis();
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
  *(dr+ 2) = distance3;
  /*SerialBT.println(distance1);
  delay(2000);
  SerialBT.println(distance2);
  delay(2000);
  SerialBT.println(distance3);
  delay(2000);*/
}

void moveForward() {
  ledcWrite(CHANNEL1, MAX);  
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MAX - 40);  
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
  ledcWrite(CHANNEL4, MAX - 40);
}

void moveLeft() {
  ledcWrite(CHANNEL1, MIN);   
  ledcWrite(CHANNEL2, MAX);
  ledcWrite(CHANNEL3, MAX - 40);  
  ledcWrite(CHANNEL4, MIN);
}

// PID para el sensor VL6180X
void pidControl() {
  float targetDistance = 85;  // Distancia objetivo
  float error, derivative, pidOutput;

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
    ledcWrite(CHANNEL3, speedRight- 40); 
    ledcWrite(CHANNEL4, MIN);       
  } else {
    ledcWrite(CHANNEL3, MIN);       
    ledcWrite(CHANNEL4, abs(speedRight));  
  }
}
void rotate90Right() {
  sensor.getRotation(&gx, &gy, &gz);
  girosc_ang_z = gz;
  float target_angle = gz - 45;  // Ángulo de rotación deseado
  while (girosc_ang_z > target_angle) {
    sensor.getRotation(&gx, &gy, &gz);
    gz -= gz_cal;

    unsigned long currentTime = millis();
    elapsedTime = (currentTime - tiempo_prev) / 1000.0;  // Convertir a segundos
    tiempo_prev = currentTime;

    girosc_ang_z += (gz / 131.0) * elapsedTime;
    Serial.println(girosc_ang_z);
    moveRight();  // Mueve los motores para girar a la derecha
    delay(500);
  }
  stopMotors();
  delay(2000);
  
}
void rotate90Left() {
  sensor.getRotation(&gx, &gy, &gz);
  girosc_ang_z = gz;
  float target_angle = gz - 40;  // Ángulo de rotación deseado
  while (girosc_ang_z > target_angle) {
    sensor.getRotation(&gx, &gy, &gz);
    gz -= gz_cal;

    unsigned long currentTime = millis();
    elapsedTime = (currentTime - tiempo_prev) / 1000.0;  // Convertir a segundos
    tiempo_prev = currentTime;

    girosc_ang_z += (gz / 131.0) * elapsedTime;
    moveLeft();  // Mueve los motores para girar a la izquierda
    delay(500);
  }
  stopMotors();
  delay(2000); 
}
void rotate180Left() {
  sensor.getRotation(&gx, &gy, &gz);
  girosc_ang_z = gz;
  float target_angle = gz - 90 ;  // Ángulo de rotación deseado
  while (girosc_ang_z > target_angle) {
    sensor.getRotation(&gx, &gy, &gz);
    gz -= gz_cal;

    unsigned long currentTime = millis();
    elapsedTime = (currentTime - tiempo_prev) / 1000.0;  // Convertir a segundos
    tiempo_prev = currentTime;

    girosc_ang_z += (gz / 131.0) * elapsedTime;
    moveLeft();  // Mueve los motores para girar a la izquierda
    delay(500);
  }
  stopMotors();

  delay(2000);
  
}

void calibrateMPU6050() {
  const int calibrations = 2000;
  float sumGz = 0;

  Serial.println("Calibrando el MPU6050...");
  for (int i = 0; i < calibrations; i++) {
    sensor.getRotation(&gx, &gy, &gz);
    sumGz += gz;
    delay(3);
  }
  gz_cal = sumGz / calibrations;
  Serial.println("Calibración completada.");
}
enum estadosMovimiento
{
  PISODETECTADO,
  MOVEFORWARD,
  ROTATEL90,
  ROTATER90,
  ROTATE180
};

void movements() { 
    int8_t piso = digitalRead(SL);
    estadosMovimiento state; 
    measurement(distance);
    if (piso) {  
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
        state = ROTATER90;   
    }
    else if (distance[0] < 85 && distance[1] > 85 && distance[2] > 85 ) {   //detectan izquierda 
        state = ROTATER90 ;   
    }
    else if (distance[0] > 85 && distance[1] < 85 && distance[2] > 85 ) {   //detectan medio 
        state = ROTATER90;   
    }
    else if (distance[0] < 85 && distance[1] < 85 && distance[2] < 85) {   //detectan todos 
        state = ROTATE180 ;  
    }
    else if (distance[0] > 85 && distance[1] > 85 && distance[2] > 85 ) {   // no detecta ninguno 
        state = MOVEFORWARD ;   
    }
    switch (state) {
        case PISODETECTADO: 
            stopMotors(); 
            break; 
        case MOVEFORWARD: 
            moveForward();
            pidControl();
            break;
        case ROTATEL90:  
            rotate90Left();
            break;
        case ROTATER90:  
            rotate90Right();
            break;
        case ROTATE180: 
            rotate180Left();
            break; 
    }
} 
void start() {
  Serial.begin(115200);
  Wire.begin();
}
void setup() {
 // SerialBT.begin("Esp32");
  start();
  start_sensors();
  reposo();
}

void loop() {
// SerialBT.println("tonto");
 movements();
}
