// Librerias I2C para controlar el mpu6050
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo del estado de AD0
MPU6050 sensor;

// Valor RAW del giroscopio en el eje z
int16_t gz;
int16_t gy;
int16_t gx;
float gz_cal=0;
long tiempo_prev, dt;
float elapsedTime;
float girosc_ang_z;
float girosc_ang_z_prev;

#define FREQ 1000
#define RES 8
#define MAX 90
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

bool boton_presionado = false;

void setup() {
    Serial.begin(115200);    //Iniciar puerto serial
    Wire.begin();           //Iniciar I2C  
    sensor.initialize();    //Iniciar el sensor
   
    if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
    else Serial.println("Error al iniciar el sensor");
     calibrateMPU6050();
    tiempo_prev = millis();
  
    pinMode(PIN_BOTON_START, INPUT_PULLUP);  // Configura el botón como entrada

    ledcAttachPin(PIN_MOTOR_A1, CHANNEL1);
    ledcSetup(CHANNEL1, FREQ, RES);
    ledcAttachPin(PIN_MOTOR_A2, CHANNEL2);
    ledcSetup(CHANNEL2, FREQ, RES);
    ledcAttachPin(PIN_MOTOR_B1, CHANNEL3);
    ledcSetup(CHANNEL3, FREQ, RES);
    ledcAttachPin(PIN_MOTOR_B2, CHANNEL4);
    ledcSetup(CHANNEL4, FREQ, RES);
}

void reposo() {
  // Espera hasta que el botón sea presionado
  while (!boton_presionado) {
    if (digitalRead(PIN_BOTON_START) == LOW) { // Verificar si el botón está presionado (LOW)
      boton_presionado = true;
      delay(200);  // Evitar debounce con un retraso
      Serial.println(boton_presionado);
    }
    Serial.println(boton_presionado);
  }
}

void moveForward() {
    ledcWrite(CHANNEL1, MAX);  
    ledcWrite(CHANNEL2, MIN);
    ledcWrite(CHANNEL3, MAX - 40);  
    ledcWrite(CHANNEL4, MIN );
}

void stopMotors() {
    ledcWrite(CHANNEL1, MIN);
    ledcWrite(CHANNEL2, MIN);
    ledcWrite(CHANNEL3, MIN);
    ledcWrite(CHANNEL4, MIN);
}

void turnRight() {
    ledcWrite(CHANNEL1, MIN);  
    ledcWrite(CHANNEL2, MAX);
    ledcWrite(CHANNEL3, MAX -40);  
    ledcWrite(CHANNEL4, MIN) ;
}

void turnLeft() {
    ledcWrite(CHANNEL1, MAX);  
    ledcWrite(CHANNEL2, MIN);
    ledcWrite(CHANNEL3, MIN);  
    ledcWrite(CHANNEL4, MAX - 40 );
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
    turnRight();  // Mueve los motores para girar a la derecha
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
    Serial.println(girosc_ang_z);
    turnLeft();  // Mueve los motores para girar a la izquierda
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


void loop() {
    reposo(); // Espera a que el botón sea presionado para iniciar el código
    moveForward();
    delay(1000);
    stopMotors();
    delay(10); 
    rotate90Right();
    delay(1000); 
    rotate90Left();
    delay(1000); // Pequeña pausa después de girar*/
}
