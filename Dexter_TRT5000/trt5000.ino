#define SL = 12;  
int sensorState = 0;         
int prestate = 0;            

void setup() {
  pinMode(pin_SENSOR, INPUT);  
}

void loop() {
  sensorState = digitalRead(pin_SENSOR);  // Lee el estado del sensor

  if (sensorState == HIGH && prestate == 0) {  
    if (analogRead(pin_SENSOR) < 512) {  
      stopMotors(); 
    } else {  
     int state = 0;   // Apaga el LED para indicar blanco
    }
    prestate = 1;  // Marca el estado del sensor como detectado
  } else if (sensorState == LOW) {
    prestate = 0;  // Restablece el estado cuando el sensor no está activo
  }
}
