#include <Wire.h>
#include "MPU6050.h"

// Parámetros del PID
#define KP 1.9          // Ganancia proporcional
#define KD 0.3          // Ganancia derivativa
#define BASE_SPEED 127  // Velocidad base de los motores (0 - 255)
#define PIN_BOTON_START 18
// Pines de los motores y canales PWM
#define FREQ 1000
#define RES 8
#define MAX 100
#define MIN 0

#define PIN_MOTOR_A1 26
#define PIN_MOTOR_A2 27
#define PIN_MOTOR_B1 13
#define PIN_MOTOR_B2 14

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

bool boton_presionado = false;
// Pines del MPU6050
MPU6050 sensor;
// Variables del giroscopio
int16_t gx, gy, gz;
float gx_cal = 0, gy_cal = 0, gz_cal = 0;

// Variables de tiempo
unsigned long previousTime = 0;
float elapsedTime;

// Variables de ángulo
float anglez = 0;
float previousAngleY = 0;

// Variables PID
float targetAngle = 0;  // Ángulo objetivo en grados
float error;
float previousError = 0;
float pidOutput;
float derivative;

void reposo() {
  // Espera hasta que el botón sea presionado
  while (!boton_presionado) {
    if (digitalRead(PIN_BOTON_START) == LOW) { // Verificar si el botón está presionado (LOW)
      boton_presionado = true;
      delay(200);  // Evitar debounce con un retraso
    }
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  sensor.initialize();

  // Configurar el pin del botón como entrada con pull-up interno
  pinMode(PIN_BOTON_START, INPUT_PULLUP);

  // Verificar conexión con el sensor
  if (!sensor.testConnection()) {
    Serial.println("Error al conectar con el MPU6050");
    while (1);
  }

  calibrateMPU6050();

  // Configuración de los pines y canales de los motores
  ledcAttachPin(PIN_MOTOR_A1, CHANNEL1);
  ledcSetup(CHANNEL1, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_A2, CHANNEL2);
  ledcSetup(CHANNEL2, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_B1, CHANNEL3);
  ledcSetup(CHANNEL3, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_B2, CHANNEL4);
  ledcSetup(CHANNEL4, FREQ, RES);

  previousTime = millis();
}

void loop() {
  reposo(); // Espera a que el botón sea presionado para iniciar el código

  unsigned long currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;  // Convertir a segundos
  previousTime = currentTime;

  readMPU6050();

  // Cálculo del ángulo actual en el eje Y
  anglez += (gz / 131.0) * elapsedTime;

  // Cálculo del error y derivada
  error = targetAngle - anglez;
  derivative = (error - previousError) / elapsedTime;

  // Calcular la salida PID
  pidOutput = (KP * error) + (KD * derivative);
  previousError = error;

  // Calcular velocidades de los motores ajustadas con la señal PID
  int motorSpeedLeft = constrain(BASE_SPEED - pidOutput, MIN, MAX);
  int motorSpeedRight = constrain(BASE_SPEED + pidOutput, MIN, MAX);

  // Controlar los motores con las velocidades calculadas
  controlMotor(motorSpeedLeft, motorSpeedRight);

  // Imprimir valores en el Serial Monitor
  Serial.print("Ángulo z: ");
  Serial.print(anglez);
  Serial.print(" grados | ");
  
  Serial.print("PID Output: ");
  Serial.print(pidOutput);
  Serial.print(" | ");

  Serial.print("Velocidad Motor Izquierdo: ");
  Serial.print(motorSpeedLeft);
  Serial.print(" | ");

  Serial.print("Velocidad Motor Derecho: ");
  Serial.println(motorSpeedRight);

  delay(100);  // Retardo de 100 ms
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

void readMPU6050() {
  sensor.getRotation(&gx, &gy, &gz);
  gz -= gz_cal;
}

void controlMotor(int speedLeft, int speedRight) {
  // Controla el motor izquierdo
  if (speedLeft > 0) {
    ledcWrite(CHANNEL1, speedLeft); // Motor A1 adelante
    ledcWrite(CHANNEL2, MIN);       // Motor A2 apagado
  } else {
    ledcWrite(CHANNEL1, MIN);       // Motor A1 apagado
    ledcWrite(CHANNEL2, abs(speedLeft)); // Motor A2 en reversa
  }

  // Controla el motor derecho
  if (speedRight > 0) {
    ledcWrite(CHANNEL3, speedRight); // Motor B1 adelante
    ledcWrite(CHANNEL4, MIN);       // Motor B2 apagado
  } else {
    ledcWrite(CHANNEL3, MIN);       // Motor B1 apagado
    ledcWrite(CHANNEL4, abs(speedRight)); // Motor B2 en reversa
  }
}
