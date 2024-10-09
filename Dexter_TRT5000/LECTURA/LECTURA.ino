// Definir el pin de salida del sensor
int TCRT = 4;

void setup() {
  // Inicializar comunicación serie
  Serial.begin(9600);
  
  // Configurar el pin del sensor como entrada
  pinMode(TCRT, INPUT);
}

void loop() {
  // Leer el valor del sensor
  int sensorValue = digitalRead(TCRT);
  
  // Si el sensor detecta algo (una línea oscura o un objeto cercano)
  if (sensorValue == LOW) {
    Serial.println("Objeto detectado o línea negra");
  } else {
    Serial.println("No se detecta nada o línea clara");
  }
  
  // Pequeño retraso para evitar sobrecarga de lecturas
  delay(100);
}
