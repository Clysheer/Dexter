#include <Wire.h>                  // Librería para comunicación I2C
#include <Adafruit_MotorShield.h>  // Librería para el Motor Shield de Adafruit
#include "MPU6050.h"              // Librería para el sensor MPU6050

#define SCL 22
#define SDA 21
// Parámetros del PID (ajustar según sea necesario)
#define KP 11            // Ganancia proporcional
#define KI 0.09          // Ganancia integral
#define KD 10            // Ganancia derivativa

#define MTRSPD 127       // Velocidad base de los motores (ajustar según sea necesario)

// Crear una instancia del Motor Shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Declarar los motores conectados al Motor Shield
Adafruit_DCMotor *UL = AFMS.getMotor(4);  // Motor izquierdo (trasero)
Adafruit_DCMotor *UR = AFMS.getMotor(3);  // Motor derecho (trasero)

// Crear una instancia del sensor MPU6050
MPU6050 sensor;

// Variables para el control PID
float target = 0;          // Objetivo del ángulo (normalmente 0 para equilibrar)
float error = 0;           // Error entre el objetivo y el ángulo actual
float integral = 0;       // Acumulador para la parte integral del PID
float derivative = 0;     // Tasa de cambio del error para la parte derivativa del PID
float last_error = 0;     // Error de la iteración anterior
float angle = 0;          // Ángulo calculado para el control PID

// Variables para el MPU-6050
int16_t gyro_x, gyro_y, gyro_z;                // Datos del giroscopio
long gyro_x_cal, gyro_y_cal, gyro_z_cal;   // Valores de calibración del giroscopio
boolean set_gyro_angles = false;            // Flag para indicar si se ha iniciado el ajuste de ángulos del giroscopio

void setup() {
 Wire1.begin(SDA, SCL);   // Iniciar la comunicación I2C
  AFMS.begin();    // Iniciar el Motor Shield
  
  sensor.initialize();  // Inicializar el sensor MPU6050
  
  // Verificar la conexión del sensor
  if (sensor.testConnection()) {
    Serial.println("Sensor iniciado correctamente");  // Mensaje si la conexión es exitosa
  } else {
    Serial.println("Error al iniciar el sensor");     // Mensaje si la conexión falla
  }

  // Calibrar el MPU-6050
  gyro_x_cal = gyro_y_cal = gyro_z_cal = 0;  // Inicializar valores de calibración
  for (int cal_int = 0; cal_int < 1000; cal_int++) {  // Realizar la calibración 1000 veces
    read_mpu_6050_data();  // Leer los datos del MPU-6050
    gyro_x_cal += gyro_x;  // Acumular el valor del giroscopio en el eje X
    gyro_y_cal += gyro_y;  // Acumular el valor del giroscopio en el eje Y
    gyro_z_cal += gyro_z;  // Acumular el valor del giroscopio en el eje Z
    delay(3);              // Esperar 3 milisegundos para permitir la calibración
  }
  
  // Dividir la suma acumulada por el número de muestras para obtener el offset promedio
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;
  
  Serial.begin(9600);  // Iniciar el monitor serial para depuración
}

void loop() {
  read_mpu_6050_data();  // Leer datos del MPU-6050
  
  // Ajustar los datos del giroscopio con los valores de calibración
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  
  // Calcular ángulos basados en el giroscopio
  float dt = (millis() - tiempo_prev) / 1000.0; // Calcular el tiempo transcurrido en segundos
  tiempo_prev = millis();  // Actualizar el tiempo previo
  
  angle += gyro_x * 0.0000611 * dt;  // Actualizar el ángulo de pitch
  
  // Control PID
  error = target - angle;          // Calcular el error
  integral += error * dt;          // Calcular la parte integral
  derivative = (error - last_error) / dt;  // Calcular la parte derivativa
  angle = (KP * error) + (KI * integral) + (KD * derivative);  // Calcular la salida PID
  
  // Actualizar el error previo
  last_error = error;
  
  // Configurar la velocidad de los motores según la salida del PID
  UR->setSpeed(MTRSPD + angle);
  UL->setSpeed(MTRSPD - angle);
  UR->run(FORWARD);
  UL->run(FORWARD);

  // Mostrar los datos en el monitor serial
  Serial.print("Ángulo: ");
  Serial.println(angle);
  
  delay(100);  // Esperar 100 milisegundos antes de la siguiente lectura
}

void read_mpu_6050_data() {
  // Leer los datos del giroscopio desde el MPU-6050
  sensor.getRotation(&gyro_x, &gyro_y, &gyro_z);
}
