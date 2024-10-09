#include <Arduino.h>
#include "MPU6050.h"
#include "Wire.h"

#define SCL 22
#define SDA 21

#define R90  90
#define GZN  131.0
#define DTN  1000.0 
#define TIME 10

MPU6050 sensor;

void rotate(int *L, float *G){ // función para que rote el MPU6050
    float initialAngle = *(G);    //angulo inicial se guearda en G
    while (*(G) - initialAngle < R90) {
        sensor.getRotation(L, L+1, L+2);
        *(L+5) = millis() - *(L+4);
        *(L+4) = millis();
        *(G) = (*(L+2) / GZN) * *(L+5) / DTN + *(G+1);
        *(G+1) = *(G);
    }
    delay(TIME);
} 


void setup() {
    Wire.begin();
    sensor.initialize();
    if (!sensor.testConnection()) {
        while (true); // Si falla la conexión, se queda aquí
    }
}

void loop() {
    int L[6] = {0};     // Array para almacenar las lecturas del giroscopio
    float G[2] = {0.0}; // Array para almacenar los ángulos

    rotate(L, G); // Ejecuta la rotación de 90 grados
    delay(1000);  // Pausa antes de la siguiente rotación
}
