#include <Arduino.h>
#include "MPU6050.h"
#include "Wire.h"

#define R90  90
#define GZN  131.0
#define DTN  1000.0 
#define TIME 10
#define FREQ 1000
#define RES 8
#define HIGH 1
#define LOW 0

#define SCL 22
#define SDA 21

#define PIN_MOTOR_A1 9
#define PIN_MOTOR_B1 16
#define PIN_MOTOR_B2 17
#define PIN_MOTOR_A2 18 

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

MPU6050 sensor;

void rotate(int *L, float *G, float angle){ // Función para que rote el MPU6050
    float initialAngle = *(G);
    while (*(G) - initialAngle < R90) {
        sensor.getRotation(L, L+1, L+2);
        *(L+5) = millis() - *(L+4);
        *(L+4) = millis();
        *(G) = (*(L+2) / GZN) * *(L+5) / DTN + *(G+1);
        *(G+1) = *(G);
    }
    delay(TIME);
}

 sensor.getRotation(&gx, &gy, &gz);
  dt = millis() - tiempo_prev;                      
  tiempo_prev = millis();                          
  girosc_ang_x = (gx / 131.0) * (dt / 1000.0) + girosc_ang_x_prev;
  girosc_ang_y = (gy / 131.0) * (dt / 1000.0) + girosc_ang_y_prev;
  girosc_ang_x_prev = girosc_ang_x;
  girosc_ang_y_prev = girosc_ang_y;


void moveLeft() {
    ledcWrite(CHANNEL1, LOW);   
    ledcWrite(CHANNEL2, HIGH);
    ledcWrite(CHANNEL3, HIGH);  
    ledcWrite(CHANNEL4, LOW);
}

void moveRight() {
    ledcWrite(CHANNEL1, HIGH); 
    ledcWrite(CHANNEL2, LOW);
    ledcWrite(CHANNEL3, LOW);   
    ledcWrite(CHANNEL4, HIGH);
}

void stopMotors() {
    ledcWrite(CHANNEL1, LOW);
    ledcWrite(CHANNEL2, LOW);
    ledcWrite(CHANNEL3, LOW);
    ledcWrite(CHANNEL4, LOW);
}

void rotateL90() {
    int L[6] = {0};     // Array para almacenar las lecturas del giroscopio
    float G[2] = {0.0}; // Array para almacenar los ángulos
    
    moveLeft();
    rotate(L, G);
    stopMotors();
}

void rotateR90() {
    int L[6] = {0};     // Array para almacenar las lecturas del giroscopio
    float G[2] = {0.0}; // Array para almacenar los ángulos
    
    moveRight();
    rotate(L, G);
    stopMotors();
}

void setup() {
    Wire1.begin(SDA, SCL);
    sensor.initialize();
    if (!sensor.testConnection()) {
        while (true); // Si falla la conexión, se queda aquí
    }

    // Configuración de los pines y canales de los motores
    ledcAttachPin(PIN_MOTOR_A1, CHANNEL1);
    ledcSetup(CHANNEL1, FREQ, RES);
    ledcAttachPin(PIN_MOTOR_A2, CHANNEL2);
    ledcSetup(CHANNEL2, FREQ, RES);
    ledcAttachPin(PIN_MOTOR_B1, CHANNEL3);
    ledcSetup(CHANNEL3, FREQ, RES);
    ledcAttachPin(PIN_MOTOR_B2, CHANNEL4);
    ledcSetup(CHANNEL4, FREQ, RES);
}

void loop() {
    rotateL90();  // Ejecuta la rotación a la izquierda 90 grados
    delay(1000);  // Pausa antes de la siguiente acción
    
    rotateR90();  // Ejecuta la rotación a la derecha 90 grados
    delay(1000);  // Pausa antes de la siguiente acción
}
