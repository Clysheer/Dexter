#include <Arduino.h>
#include "MPU6050.h"
#include "Wire.h"

// Definiciones de constantes
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

// Variables del giroscopio
int16_t gx, gy, gz;
unsigned long tiempo_prev = 0;
float girosc_ang_z = 0;
float girosc_ang_z_prev = 0;

void calibrateMPU6050() {
    const int calibrations = 2000;
    float sumGz = 0;

    Serial.println("Calibrando el MPU6050...");
    for (int i = 0; i < calibrations; i++) {
        sensor.getRotation(&gx, &gy, &gz);
        sumGz += gz;
        delay(3);
    }
    gz = sumGz / calibrations;
    Serial.println("Calibración completada.");
}

void rotate(float angle) { // Función para rotar basándose en el ángulo del giroscopio
    unsigned long currentTime;
    float dt;
    
    // Guardar el ángulo inicial
    float initialAngle = girosc_ang_z;
    
    // Mover en dirección deseada
    moveLeft(); // O moveRight() dependiendo de la dirección deseada
    
    while (abs(girosc_ang_z - initialAngle) < angle) {
        sensor.getRotation(&gx, &gy, &gz);
        
        currentTime = millis();
        dt = (currentTime - tiempo_prev) / 1000.0; // Tiempo en segundos
        tiempo_prev = currentTime;
        
        // Actualizar ángulo giroscópico
        girosc_ang_z += (gz / GZN) * dt;
        
        // Imprimir valores en el monitor serial
        Serial.print("Ángulo Z: ");
        Serial.println(girosc_ang_z);
        
        delay(10); // Pequeño retraso para evitar lecturas demasiado rápidas
    }
    
    stopMotors();
    delay(TIME); // Pausa después de la rotación
}

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
    rotate(R90); // Ejecuta la rotación a la izquierda 90 grados
}

void rotateR90() {
    rotate(R90); // Ejecuta la rotación a la derecha 90 grados
}

void setup() {
    Serial.begin(115200); // Inicializa el monitor serial
    Wire.begin(SDA, SCL);
    sensor.initialize();
    if (!sensor.testConnection()) {
        Serial.println("Error al conectar con el MPU6050");
        while (true); // Si falla la conexión, se queda aquí
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
}

void loop() {
    rotateL90();  // Ejecuta la rotación a la izquierda 90 grados
    delay(1000);  // Pausa antes de la siguiente acción
    
    rotateR90();  // Ejecuta la rotación a la derecha 90 grados
    delay(1000);  // Pausa antes de la siguiente acción
}
