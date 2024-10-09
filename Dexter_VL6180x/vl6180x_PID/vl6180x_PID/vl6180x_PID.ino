#include <Wire.h>
#include <Adafruit_VL6180X.h>

// Parámetros del PID
#define KP 4.0          // Ganancia proporcional
#define KD 0.1              // Ganancia derivativa
#define BASE_SPEED 127  // Velocidad base de los motores (0 - 255)
#define PIN_BOTON_START 18

// Pines de los motores y canales PWM
#define FREQ 1000
#define RES 8
#define MAX 50
#define MIN 0

#define PIN_MOTOR_A1 26
#define PIN_MOTOR_A2 27
#define PIN_MOTOR_B1 13
#define PIN_MOTOR_B2 14

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define LOX3_ADDRESS 0x30
#define SHT_LOX3 2
#define SCL 22
#define SDA 21
#define PWM 14

bool boton_presionado = false;
// Sensor VL6180X
Adafruit_VL6180X lox3 = Adafruit_VL6180X();
uint16_t distance3 = 0;

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
  Serial.begin(115200);

  // Configurar el pin del botón como entrada con pull-up interno
  pinMode(PIN_BOTON_START, INPUT_PULLUP);

  // Configuración de los pines y canales de los motores
  ledcAttachPin(PIN_MOTOR_A1, CHANNEL1);
  ledcSetup(CHANNEL1, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_A2, CHANNEL2);
  ledcSetup(CHANNEL2, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_B1, CHANNEL3);
  ledcSetup(CHANNEL3, FREQ, RES);
  ledcAttachPin(PIN_MOTOR_B2, CHANNEL4);
  ledcSetup(CHANNEL4, FREQ, RES);

  // Inicializar el sensor de distancia
  pinMode(SHT_LOX3, OUTPUT);
  setID();
}
void pid(){
  float targetDistance = 50;  // Distancia objetivo en mm
  float previousError = 0;  
  float pidOutput;
  float derivative;
  float error;
  readDistance(lox3, distance3);
  Serial.print("Distancia: ");
  Serial.println(distance3);
  error = targetDistance - distance3;
  derivative = error - previousError;
  pidOutput = (KP * error) + (KD * derivative);
  previousError = error;
  int motorSpeedLeft = constrain(BASE_SPEED + pidOutput, MIN, MAX);  // Corregido
  int motorSpeedRight = constrain(BASE_SPEED - pidOutput, MIN, MAX); // Corregido
  controlMotor(motorSpeedLeft, motorSpeedRight); 
  delay(100);  // Retardo de 100 ms
}

void loop() {
  reposo(); // Espera a que el botón sea presionado para iniciar el código
  pid();
}

void setID() {
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  digitalWrite(SHT_LOX3, HIGH);
  while (!lox3.begin()) 
    Serial.println("lox3 no se pudo iniciar!!!");
}

void readDistance(Adafruit_VL6180X &vl, uint16_t &distance) {
  uint16_t range = vl.readRange();
  distance = range;
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
