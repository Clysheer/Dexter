#define FREQ 1000
#define RES 8
#define MAX 100
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


bool boton_presionado = false;

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
    //ledcSetup(0, 5000, 8); // Canal 0, 5 kHz, 8 bits de resolución
    //ledcAttachPin(PIN_MOTOR_B2, 0); // Configuración de los pines y canales de los motores
    ledcAttachPin(PIN_MOTOR_A1, CHANNEL1);
    ledcSetup(CHANNEL1, FREQ, RES);
    ledcAttachPin(PIN_MOTOR_A2, CHANNEL2);
    ledcSetup(CHANNEL2, FREQ, RES);
    ledcAttachPin(PIN_MOTOR_B1, CHANNEL3);
    ledcSetup(CHANNEL3, FREQ, RES);
    ledcAttachPin(PIN_MOTOR_B2, CHANNEL4);
    ledcSetup(CHANNEL4, FREQ, RES);
}



void moveBackward() {
    ledcWrite(CHANNEL1, MIN);  
    ledcWrite(CHANNEL2, MAX - 50);
    ledcWrite(CHANNEL3, MIN);  
    ledcWrite(CHANNEL4, MAX);
}
void moveForward() {
  ledcWrite(CHANNEL1, MAX - 50);  
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
void moveRight() {
  ledcWrite(CHANNEL1, MAX); 
  ledcWrite(CHANNEL2, MIN);
  ledcWrite(CHANNEL3, MIN);   
  ledcWrite(CHANNEL4, MAX);
}

void moveLeft() {
  ledcWrite(CHANNEL1, MIN);   
  ledcWrite(CHANNEL2, MAX);
  ledcWrite(CHANNEL3, MAX);  
  ledcWrite(CHANNEL4, MIN);
}
void loop() {
    reposo(); // Espera a que el botón sea presionado para iniciar el código
    moveForward(); 
    delay(2000);
    stopMotors();
    delay(3000);
    moveBackward();
    delay(3000);
     // Mueve los motores hacia adelante    // Mantiene el movimiento hacia adelante por 2 segundos
}