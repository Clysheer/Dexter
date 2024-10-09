#include <Adafruit_VL6180X.h>

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

#define SHT_LOX1 2 //derecha
#define SHT_LOX2 19 // medio
#define SHT_LOX3 5 // izquierda

Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();

uint16_t distance1 = 0;
uint16_t distance2 = 0;
uint16_t distance3 = 0;

void setID() {  
  digitalWrite(SHT_LOX1, LOW);
  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  delay(10);
  
  if (!lox1.begin()) {
    Serial.println("lox1 no se pudo iniciar!!!"); 
    while (1);
  }
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);
  
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  
  if (!lox2.begin()) { 
    Serial.println("lox2 no se pudo iniciar!!!");
    while (1);
  }
  lox2.setAddress(LOX2_ADDRESS);
  delay(10); 
  
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  
  if (!lox3.begin()) {
    Serial.println("lox3 no se pudo iniciar!!!");
    while (1);
  }
  lox3.setAddress(LOX3_ADDRESS);
  delay(10); 
}

void readDistance(Adafruit_VL6180X &vl, uint16_t &distance) {
  uint16_t range = vl.readRange();
  distance = range;
}

void setup() {
  Serial.begin(115200);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  setID();
}

void loop() {
  readDistance(lox1, distance1);
  readDistance(lox2, distance2);
  readDistance(lox3, distance3);

  Serial.print("Distance 1: "); Serial.println(distance1);
  Serial.print("Distance 2: "); Serial.println(distance2);
  Serial.print("Distance 3: "); Serial.println(distance3);

  delay(500); 
}
