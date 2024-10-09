#include <Wire.h>
#include "MPU6050.h"
#include <Adafruit_VL6180X.h>

// Definición de constantes
#define KP 1.9
#define KD 0.0
#define BASE_SPEED 127
#define FREQ 1000
#define RES 8
#define MAX1 100
#define MAX2 50
#define MIN 0
#define DISTANCE_THRESHOLD 80
#define PIN_BOTON_START 18

// Pines de los motores
#define PIN_MOTOR_A1 26
#define PIN_MOTOR_A2 27
#define PIN_MOTOR_B1 13
#define PIN_MOTOR_B2 14

// Canales de PWM
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

// Pines de los sensores de distancia y MPU6050
#define SHT_LOX1 19
#define SHT_LOX2 5
#define SHT_LOX3 2
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// Variables globales
Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();

uint16_t distance1 = 0, distance2 = 0, distance3 = 0;
MPU6050 sensor;
int16_t gx, gy, gz;
float anglez = 0, gz_cal = 0;
bool boton_presionado = false;

// Variables PID
float error, previousError = 0, pidOutput, derivative;
float previousTime = 0, elapsedTime = 0;
float targetAngle = 0;  // Ángulo objetivo para el equilibrio

void reposo() {
  while (!boton_presionado) {
    if (digitalRead(PIN_BOTON_START) == LOW) { 
      boton_presionado = true;
      delay(200);  // Evitar debounce
    }
  }
}

void setup() {
  // Inicialización
  delay(15000);  // Espera antes de comenzar
  Serial.begin(921600);
  Wire.begin();
  sensor.initialize();

  // Configuración de botones, motores y sensores
  pinMode(PIN_BOTON_START, INPUT_PULLUP);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  if (!sensor.testConnection()) {
    Serial.println("Error al conectar con el MPU6050");
    while (1);
  }

  calibrateMPU6050();
  setID();  // Configura direcciones de los sensores

  // Configuración de los canales PWM de los motores
  ledcSetup(CHANNEL1, FREQ, RES);
  ledcSetup(CHANNEL2, FREQ, RES);
  ledcSetup(CHANNEL3, FREQ, RES);
  ledcSetup(CHANNEL4, FREQ, RES);

  ledcAttachPin(PIN_MOTOR_A1, CHANNEL1);
  ledcAttachPin(PIN_MOTOR_A2, CHANNEL2);
  ledcAttachPin(PIN_MOTOR_B1, CHANNEL3);
  ledcAttachPin(PIN_MOTOR_B2, CHANNEL4);

  previousTime = millis();
}

void loop() {
  reposo();  // Espera a que se presione el botón para empezar

  readDistanceSensors();
  processSensors();  // Llama al proceso de detección y actuación de motores
}

void processSensors() {
  // Detección de casos de sensores y llamada al control de motores
  int caseDetected = 0;
  if (distance3 < DISTANCE_THRESHOLD && distance1 < DISTANCE_THRESHOLD && distance2 > DISTANCE_THRESHOLD) {
    caseDetected = 1;  // Sensores derecho e izquierdo
  } else if (distance3 > DISTANCE_THRESHOLD && distance1 < DISTANCE_THRESHOLD && distance2 < DISTANCE_THRESHOLD) {
    caseDetected = 2;  // Sensor derecho
  } else if (distance3 < DISTANCE_THRESHOLD && distance1 > DISTANCE_THRESHOLD && distance2 < DISTANCE_THRESHOLD) {
    caseDetected = 3;  // Sensores izquierdo y derecho
  } else if (distance3 > DISTANCE_THRESHOLD && distance1 < DISTANCE_THRESHOLD && distance2 > DISTANCE_THRESHOLD) {
    caseDetected = 4;  // Sensor derecho solo
  }

  switch (caseDetected) {
    case 1:
      moveLeft();
      runPID();  // Corre el PID para mantener el equilibrio
      break;
    case 2:
      moveRight();
      break;
    case 3:
      stopMotors();
      break;
    case 4:
      moveForward();
      runPID();  // Corre el PID para mantener el equilibrio
      break;
    default:
      stopMotors();
      break;
  }
}

void calibrateMPU6050() {
  const int calibrations = 2000;
  float sumGz = 0;
  for (int i = 0; i < calibrations; i++) {
    sensor.getRotation(&gx, &gy, &gz);
    sumGz += gz;
    delay(3);
  }
  gz_cal = sumGz / calibrations;
}

void readMPU6050() {
  sensor.getRotation(&gx, &gy, &gz);
  gz -= gz_cal;
}

void runPID() {
  unsigned long currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  readMPU6050();
  anglez += (gz / 131.0) * elapsedTime;
  
  error = targetAngle - anglez;
  derivative = (error - previousError) / elapsedTime;

  pidOutput = (KP * error) + (KD * derivative);
  previousError = error;

  int motorSpeedLeft = constrain(BASE_SPEED - pidOutput, MIN, MAX1);
  int motorSpeedRight = constrain(BASE_SPEED + pidOutput, MIN, MAX2);

  controlMotor(motorSpeedLeft, motorSpeedRight);
}

void readDistanceSensors() {
  readDistance(lox1, distance1);
  readDistance(lox2, distance2);
  readDistance(lox3, distance3);
}

void readDistance(Adafruit_VL6180X &vl, uint16_t &distance) {
  distance = vl.readRange();
}

void controlMotor(int speedLeft, int speedRight) {
  // Controla el motor izquierdo
  if (speedLeft > 0) {
    ledcWrite(CHANNEL1, speedLeft);
    ledcWrite(CHANNEL2, MIN);
  } else {
    ledcWrite(CHANNEL1, MIN);
    ledcWrite(CHANNEL2, abs(speedLeft));
  }

  // Controla el motor derecho
  if (speedRight > 0) {
    ledcWrite(CHANNEL3, speedRight);
    ledcWrite(CHANNEL4, MIN);
  } else {
    ledcWrite(CHANNEL3, MIN);
    ledcWrite(CHANNEL4, abs(speedRight));
  }
}

void moveForward() {
  ledcWrite(CHANNEL1, MAX1);
  ledcWrite(CHANNEL2, LOW);
  ledcWrite(CHANNEL3, MAX2);
  ledcWrite(CHANNEL4, LOW);
}

void moveRight() {
  ledcWrite(CHANNEL1, MAX1);
  ledcWrite(CHANNEL2, LOW);
  ledcWrite(CHANNEL3, LOW);
  ledcWrite(CHANNEL4, MAX2);
}

void moveLeft() {
  ledcWrite(CHANNEL1, LOW);
  ledcWrite(CHANNEL2, MAX1);
  ledcWrite(CHANNEL3, MAX2);
  ledcWrite(CHANNEL4, LOW);
}

void stopMotors() {
  ledcWrite(CHANNEL1, LOW);
  ledcWrite(CHANNEL2, LOW);
  ledcWrite(CHANNEL3, LOW);
  ledcWrite(CHANNEL4, LOW);
}

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

  if (!lox1.begin(&Wire)) while (1);
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);

  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if (!lox2.begin(&Wire)) while (1);
  lox2.setAddress(LOX2_ADDRESS);

  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  if (!lox3.begin(&Wire)) while (1);
  lox3.setAddress(LOX3_ADDRESS);
}
