#include <Adafruit_VL6180X.h>

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

#define SHT_LOX1 19
#define SHT_LOX2 5
#define SHT_LOX3 2
#define MAX1 100
#define MAX2 50
#define SCL 22
#define SDA 21

#define PIN_MOTOR_A1 26
#define PIN_MOTOR_A2 27
#define PIN_MOTOR_B1 13
#define PIN_MOTOR_B2 14

#define FREQ 1000
#define RES 8
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define DISTANCE_THRESHOLD 80

Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();

uint16_t distance1 = 0;
uint16_t distance2 = 0;
uint16_t distance3 = 0;

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

  if (!lox1.begin(&Wire1)) while (1);
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);
  
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if (!lox2.begin(&Wire1)) while (1);
  lox2.setAddress(LOX2_ADDRESS);

  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  if (!lox3.begin(&Wire1)) while (1);
  lox3.setAddress(LOX3_ADDRESS);
}

void readDistance(Adafruit_VL6180X &vl, uint16_t &distance) {
  uint16_t range = vl.readRange();
  distance = range;
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

void moveBack() {
  ledcWrite(CHANNEL1, LOW);   
  ledcWrite(CHANNEL2, MAX1);
  ledcWrite(CHANNEL3, LOW;  
  ledcWrite(CHANNEL4, MAX2);
}


void stopMotors() {
  ledcWrite(CHANNEL1, LOW);
  ledcWrite(CHANNEL2, LOW);
  ledcWrite(CHANNEL3, LOW);
  ledcWrite(CHANNEL4, LOW);
}

void setup() {
  Serial.begin(921600);

  // Inicializar I2C
  Wire1.begin(SDA, SCL);

  // Inicializar motores
  ledcSetup(CHANNEL1, FREQ, RES);
  ledcSetup(CHANNEL2, FREQ, RES);
  ledcSetup(CHANNEL3, FREQ, RES);
  ledcSetup(CHANNEL4, FREQ, RES);
  
  ledcAttachPin(PIN_MOTOR_A1, CHANNEL1);
  ledcAttachPin(PIN_MOTOR_A2, CHANNEL2);
  ledcAttachPin(PIN_MOTOR_B1, CHANNEL3);
  ledcAttachPin(PIN_MOTOR_B2, CHANNEL4);

  // Configuración de los sensores
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  setID();
}

void loop() {
  // Leer distancias de los sensores
  readDistance(lox1, distance1);
  readDistance(lox2, distance2);
  readDistance(lox3, distance3);

 /* // Imprimir distancias en el monitor serie
  Serial.print("Distancia Sensor Medio: ");
  Serial.println(distance1);
  Serial.print("Distancia Sensor Derecho: ");
  Serial.println(distance2);
  Serial.print("Distancia Sensor Izquierdo: ");
  Serial.println(distance3);*/

  // Lógica para los motores según las distancias

  // Detecta los sensores derecho
  if (distance3 < DISTANCE_THRESHOLD && distance1 < DISTANCE_THRESHOLD && distance2 > DISTANCE_THRESHOLD) {
     Serial.println("derecho");
     moveLeft();
  }
  // Detecta solo el sensor izquierdo
  else if (distance3 > DISTANCE_THRESHOLD && distance1 < DISTANCE_THRESHOLD && distance2 < DISTANCE_THRESHOLD) {
    Serial.println("izquierdo solo");
    moveRight();
  }
  // Detecta los sensores izquierdo y derecho
  else if (distance3 < DISTANCE_THRESHOLD && distance1 > DISTANCE_THRESHOLD && distance2 < DISTANCE_THRESHOLD) {
    Serial.println("lzquierdo y derecho");
    stopMotors();
  }
  // Detecta solo el sensor derecho
  else if (distance3 > DISTANCE_THRESHOLD && distance1 < DISTANCE_THRESHOLD && distance2 > DISTANCE_THRESHOLD) {
    Serial.println("derecho solo");
    moveForward();
  }
  // Detecta los sensores central y derecho, pero no el izquierdo
  else if (distance3 < DISTANCE_THRESHOLD && distance1 > DISTANCE_THRESHOLD && distance2 > DISTANCE_THRESHOLD) {
    Serial.println("central y derecho");
    moveLeft();
  }
  // Detecta los sensores izquierdo y derecho, pero no el central
  else if (distance3 > DISTANCE_THRESHOLD && distance1 < DISTANCE_THRESHOLD && distance2 > DISTANCE_THRESHOLD) {
    Serial.println("sensor izquierdo y derecho");
    moveBack();
  }
  // Ningún sensor detecta
  else if (distance3 < DISTANCE_THRESHOLD && distance1 < DISTANCE_THRESHOLD && distance2 < DISTANCE_THRESHOLD) {
    Serial.println("Detener motores");
    stopMotors();
  }
  // Todos los sensores detectan
  else if (distance3 > DISTANCE_THRESHOLD && distance2 > DISTANCE_THRESHOLD && distance3 > DISTANCE_THRESHOLD) {
    Serial.println("todos detectan");
    moveRight();
  }

  delay(500);
}
