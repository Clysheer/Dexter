#include <Adafruit_VL6180X.h>

#define LOX1_ADDRESS 0x30

#define SHT_LOX1 2

#define SCL 22
#define SDA 21
#define PWM 14
Adafruit_VL6180X lox1 = Adafruit_VL6180X();

uint16_t distance1 = 0;

void setID() {
  digitalWrite(SHT_LOX1, LOW);
  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  while (!lox1.begin(&Wire1)) 
  Serial.println("lox1 no sepudo iniciar!!!");
}

void readDistance(Adafruit_VL6180X &vl, uint16_t &distance) {
  uint16_t range = vl.readRange();
  distance = range;
}
 
void setup() {
  Serial.begin(921600);
  Wire1.begin(SDA,SCL);
  pinMode(SHT_LOX1, OUTPUT);
  setID();
}

void loop() {
  readDistance(lox1, distance1);
}
