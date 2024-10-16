#include "BluetoothSerial.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_VL6180X.h>
 
BluetoothSerial SerialBT;

// Parámetros del PID
#define KP 4.0         // Ganancia proporcional
#define KD 0.1          // Ganancia derivativa
#define BASE_SPEED 127  // Velocidad base de los motores (0 - 255)
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

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

#define SHT_LOX1 19 // medio
#define SHT_LOX2 5 //izquierda
#define SHT_LOX3 2 //derecha

#define BEGIN 115200
#define R180 180
#define R90  90
#define GZN  131.0
#define DTN  1000.0
#define SGM 2
#define DM 3
#define LM 5
#define SL 12 // Sensor de piso

#define DMAXP 55
#define DMAXD 85
#define DMAXI 80
#define DMAXM 100
#define DISTANCE1 0 
#define DISTANCE2 1 //
#define DISTANCE3 2 //izquierda
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
  float angleZ = 0;  

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
}

void moveForward() {
  ledcWrite(CHANNEL1, MAX -45);  
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
void moveLeft() {
  ledcWrite(CHANNEL1, MAX); 
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MIN);   
  ledcWrite(CHANNEL4, MAX);
}

void moveRight() {
  ledcWrite(CHANNEL1, MIN);   
  ledcWrite(CHANNEL2, MAX);
  ledcWrite(CHANNEL3, MAX);  
  ledcWrite(CHANNEL4, MIN);
}

// PID para el sensor VL6180X
void pidControl() {
  float error, derivative, pidOutput;

  error = DMAXP - distance[DISTANCE2];
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
    ledcWrite(CHANNEL1, speedLeft - 45);  
    ledcWrite(CHANNEL2, MIN);       
  } else {
    ledcWrite(CHANNEL1, MIN);       
    ledcWrite(CHANNEL2, abs(speedLeft)- 45);  
  }

  if (speedRight > 0) {
    ledcWrite(CHANNEL3, speedRight ); 
    ledcWrite(CHANNEL4, MIN);       
  } else {
    ledcWrite(CHANNEL3, MIN);       
    ledcWrite(CHANNEL4, abs(speedRight) );  
  }
}

// Función para rotar a un ángulo específico
void RRight(float degrees) {
  // Capturar el ángulo actual antes de rotar
  float targetAngle;  
  sensor.getRotation(&gx, &gy, &gz);
  angleZ += (gz - gz_cal) * (1.0 / 131.0); // Actualizar el ángulo acumulado
  targetAngle = angleZ + degrees; // Establecer el ángulo objetivo
  rotateRight();
  // Esperar hasta alcanzar el ángulo objetivo
  while (true) {
    sensor.getRotation(&gx, &gy, &gz);
    angleZ += (gz - gz_cal) * (1.0 / 131.0); // Actualizar el ángulo acumulado

    // Comprobar si se ha alcanzado el ángulo objetivo
    if ((degrees > 0 && angleZ >= targetAngle) || (degrees < 0 && angleZ <= targetAngle)) {
      stopMotors();
      break;
    }

    // Imprimir el ángulo actual
    Serial.print("Ángulo actual: ");
    Serial.println(angleZ);
    delay(100); // Espera un momento para evitar sobrecargar el serial
  }
}

void RLeft(float degrees) {
  // Capturar el ángulo actual antes de rotar
  float targetAngle;  
  sensor.getRotation(&gx, &gy, &gz);
  angleZ += (gz - gz_cal) * (1.0 / 131.0); // Actualizar el ángulo acumulado
  targetAngle = angleZ + degrees; // Establecer el ángulo objetivo
  rotateLeft();
  // Esperar hasta alcanzar el ángulo objetivo
  while (true) {
    sensor.getRotation(&gx, &gy, &gz);
    angleZ += (gz - gz_cal) * (1.0 / 131.0); // Actualizar el ángulo acumulado

    // Comprobar si se ha alcanzado el ángulo objetivo
    if ((degrees > 0 && angleZ >= targetAngle) || (degrees < 0 && angleZ <= targetAngle)) {
      stopMotors();
      break;
    }

    // Imprimir el ángulo actual
    Serial.print("Ángulo actual: ");
    Serial.println(angleZ);
    delay(100); // Espera un momento para evitar sobrecargar el serial
  }
}
// Función para girar a la derecha
void rotateRight() {
  ledcWrite(CHANNEL1, MIN); 
  ledcWrite(CHANNEL2, MAX);
  ledcWrite(CHANNEL3, MAX - 55);   
  ledcWrite(CHANNEL4, MIN);
  SerialBT.println("Girando a la derecha");
}

// Función para girar a la izquierda
void rotateLeft() {
  ledcWrite(CHANNEL1, MAX); 
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MIN);   
  ledcWrite(CHANNEL4, MAX - 55);
  SerialBT.println("Girando a la izquierda");
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
    if (piso == HIGH){  
        SerialBT.println("Entrando en caso: PISODETECTADO");
        state = PISODETECTADO;  //detecta piso
    }
    else if (distance[DISTANCE3] > DMAXI && distance[DISTANCE1] > DMAXM && distance[DISTANCE2] < DMAXD  ) { 
        SerialBT.println("Entrando en caso: MOVEFORWARD");  //detecta derecha
        state =  MOVEFORWARD ;   
    }
    else if (distance[DISTANCE3] > DMAXI && distance[DISTANCE1] < DMAXM && distance[DISTANCE2] < DMAXD ) {  
        SerialBT.println("Entrando en caso: ROTATEL90");//detectan medio y derecha 
        state = ROTATEL90 ;  //rotatel90 
    }
   else if (distance[DISTANCE3] < DMAXI && distance[DISTANCE1] > DMAXM && distance[DISTANCE2] < DMAXD ) { 
        SerialBT.println("Entrando en caso: MOVEFORWARD");  //detectan izquierda y derecha  
        state =  MOVEFORWARD;  
    }
    else if (distance[DISTANCE3] < DMAXI && distance[DISTANCE1] < DMAXM && distance[DISTANCE2] > DMAXD ) { 
        SerialBT.println("Entrando en caso: ROTATER90");  //detectan medio y izquierda 
        state = ROTATER90;  //ROTATER90  
    }
    else if (distance[DISTANCE3] < DMAXI && distance[DISTANCE1] > DMAXM && distance[DISTANCE2] > DMAXD ) {
        SerialBT.println("Entrando en caso: ROTATER90");   //detectan izquierda 
        state = ROTATER90 ;   //ROTATER90 
    }
    else if (distance[DISTANCE3] > DMAXI && distance[DISTANCE1] < 85 && distance[DISTANCE2] > DMAXD ) { 
        SerialBT.println("Entrando en caso: ROTATEL90");  //detectan medio 
        state = ROTATEL90;   
    }
    else if (distance[DISTANCE3] < DMAXI && distance[DISTANCE1] < DMAXM && distance[DISTANCE2] < DMAXD) { 
        SerialBT.println("Entrando en caso: ROTATEL90");  //detectan todos 
        state = ROTATEL90;  
    }
    else if (distance[DISTANCE3] > DMAXI && distance[DISTANCE1] > DMAXM && distance[DISTANCE2] > DMAXD ) {  
        SerialBT.println("Entrando en caso: ROTATER90"); // no detecta ninguno 
        state = ROTATER90;   
    }
   switch (state) {
        case PISODETECTADO: 
            Serial.println("Entrando en caso: PISODETECTADO");
            stopMotors(); 
            break; 
        case MOVEFORWARD: 
            Serial.println("Entrando en caso: MOVEFORWARD");
            pidControl();
            break;
        case ROTATEL90:  
            Serial.println("Entrando en caso: ROTATEL90");
            sendData();
             if(distance[DISTANCE1] > 40){
              moveForward();
              delay(250);
            }
            RLeft(90);
            break;
        case ROTATER90:  
            Serial.println("Entrando en caso: ROTATER90");
            sendData(); if(distance[DISTANCE1] > 40){
              moveForward();
              delay(250);
            }
            RRight(-90);
            break;
        case ROTATE180: 
            Serial.println("Entrando en caso: ROTATE180");
            sendData();
             if(distance[DISTANCE1] > 40){
              moveForward();
              delay(250);
            }
            RLeft(180);
            break; 
    }
} 

void start() {
  Serial.begin(115200);
  Wire.begin();
}

void setup() {
  start();
  start_sensors();
  SerialBT.begin("MiRobot");  // Nombre del dispositivo Bluetooth
  reposo();
}

void loop() {
  movements();
  sendData();
}

void sendData() {
  SerialBT.print("Distancia 1: "); SerialBT.println(distance[0]); //medio
  SerialBT.print("Distancia 2: "); SerialBT.println(distance[1]); //derecha
  SerialBT.print("Distancia 3: "); SerialBT.println(distance[2]); //izquierda
  
  sensor.getRotation(&gx, &gy, &gz);
  SerialBT.print("Giro Z: "); SerialBT.println(gz - gz_cal);  // Enviar el valor corregido
  int8_t piso = digitalRead(SL);
  SerialBT.print("Sensor de piso: "); SerialBT.println(piso);
}
