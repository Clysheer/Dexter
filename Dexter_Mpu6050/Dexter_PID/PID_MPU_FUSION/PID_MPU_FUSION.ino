#include <Wire.h>                  // Librería para comunicación I2C
#include <Adafruit_MotorShield.h>  // Librería para el Motor Shield de Adafruit
#include "MPU6050.h"              // Librería para el sensor MPU6050

// Parámetros del PID (ajustar según sea necesario)
#define KP 11            // Ganancia proporcional
#define KI 0.09          // Ganancia integral
#define KD 10            // Ganancia derivativa

#define VPID 6;
#define VMPU 9;
#define MTRSPD 127       // Velocidad base de los motores (ajustar según sea necesario)

// Crear una instancia del Motor Shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Declarar los motores conectados al Motor Shield
Adafruit_DCMotor *UL = AFMS.getMotor(4);  // Motor izquierdo (trasero)
Adafruit_DCMotor *UR = AFMS.getMotor(3);  // Motor derecho (trasero)

// Crear una instancia del sensor MPU6050
MPU6050 sensor;

struct{
float target = 0;          // Objetivo del ángulo (normalmente 0 para equilibrar)
float error = 0;           // Error entre el objetivo y el ángulo actual
float integral = 0;       // Acumulador para la parte integral del PID
float derivative = 0;     // Tasa de cambio del error para la parte derivativa del PID
float last_error = 0;     // Error de la iteración anterior
float angle = 0;          // Ángulo calculado para el control PID
} VariablesControlPid;
VariablesControlPid VaCoPis[VPID];


struct{
int gx, gy, gz;                // Datos del giroscopio
long gx_cal, gy_cal, gz_cal;   // Valores de calibración del giroscopio
long tiempo_prev;                          // Tiempo previo
float girosc_ang_x_prev, girosc_ang_y_prev; // Ángulos previos del giroscopio
} VariablesMPU;
VariablesMPU VaMpu[VPID];

void setup(*VMPU) {
  Serial.begin(57600);  // Iniciar el puerto serial a 57600 baudios
  Wire.begin();         // Iniciar la comunicación I2C
  AFMS.begin();         // Iniciar el Motor Shield
  sensor.initialize();  // Inicializar el sensor MPU6050

  gx_cal = gy_cal = gz_cal = 0;  // Inicializar valores de calibración
  for (int cal_int = 0; cal_int < 1000; cal_int++) {  // Realizar la calibración 1000 veces
    read_mpu_6050_data();  // Leer los datos del MPU-6050
    gx_cal += gx;  // Acumular el valor del giroscopio en el eje X
    gy_cal += gy;  // Acumular el valor del giroscopio en el eje Y
    gz_cal += gz;  // Acumular el valor del giroscopio en el eje Z
    delay(3);              // Esperar 3 milisegundos para permitir la calibración
  }
  
  // Dividir la suma acumulada por el número de muestras para obtener el offset promedio
  gx_cal /= 1000;
  gy_cal /= 1000;
  gz_cal /= 1000;

  tiempo_prev = millis(); // Guardar el tiempo actual
}

void loop() {
  read_mpu_6050_data();  // Leer datos del MPU-6050
  
  // Ajustar los datos del giroscopio con los valores de calibración
  gx -= gx_cal;
  gy -= gy_cal;
  gz -= gz_cal;
  
  // Calcular ángulos basados en el giroscopio
  float dt = (millis() - tiempo_prev) / 1000.0; // Calcular el tiempo transcurrido en segundos
  tiempo_prev = millis();  // Actualizar el tiempo previo
  
  angle += gx * 0.0000611 * dt;  // Actualizar el ángulo
  
  // Control PID
  error = target - angle;          // Calcular el error
  integral += error * dt;          // Calcular la parte integral
  derivative = (error - last_error) / dt;  // Calcular la parte derivativa
  float output = (KP * error) + (KI * integral) + (KD * derivative);  // Calcular la salida PID
  
  // Actualizar el error previo
  last_error = error;

  // Configurar la velocidad de los motores según la salida del PID
  UR->setSpeed(MTRSPD + output);
  UL->setSpeed(MTRSPD - output);
  UR->run(FORWARD);
  UL->run(FORWARD);

  // Mostrar los datos en el monitor serial
  Serial.print("Ángulo: ");
  Serial.println(angle);
  
  delay(100);  // Esperar 100 milisegundos antes de la siguiente lectura
}

void read_mpu_6050_data() {
  // Leer los datos del giroscopio desde el MPU-6050
  sensor.getRotation(&gx, &gy, &gz);
}
