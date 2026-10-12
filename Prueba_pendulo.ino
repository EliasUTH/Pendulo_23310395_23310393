// Pines Puente H
const int pinPWM = 9;   // Pin ENA (PWM)
const int pinIN1 = 8;   // Pin IN1 (Dirección)
const int pinIN2 = 7;   // Pin IN2 (Dirección)

// Pin y Configuración del Sensor de Corriente ACS712
const int pinACS712 = A0;
const float VCC = 5.0; // Voltaje de operación de Arduino
/*
Según módulo ACS712: 
5A  -> 0.185 (185 mV/A)
20A -> 0.100 (100 mV/A)
30A -> 0.066 (66 mV/A)*/
const float SENSIBILIDAD_ACS712 = 0.185; 

// Calibración tiempo-ángulo (ms por grado a velocidad base)
const float MS_POR_GRADO = 8.5; 

void setup() {
  Serial.begin(9600);
  pinMode(pinPWM, OUTPUT);
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
}

// -------------------------------------------------------------------
// FUNCIÓN AUXILIAR: Lee el sensor de corriente ACS712 y retorna Amperios
// -------------------------------------------------------------------
float leerCorrienteACS712() {
  int lecturaADC = analogRead(pinACS712);
  float voltajePin = (lecturaADC / 1023.0) * VCC;
  // El ACS712 entrega VCC/2 (2.5V) en reposo (0 Amperios)
  float corriente = (voltajePin - (VCC / 2.0)) / SENSIBILIDAD_ACS712;
  return corriente; 
}

// -------------------------------------------------------------------
// FUNCIÓN 1: Mover por Velocidad y Sentido
// - velocidad: 0 a 255 (PWM)
// - sentido: 1 (Horario), -1 (Antihorario), 0 (Detener)
// -------------------------------------------------------------------
void moverPorVelocidad(int velocidad, int sentido) {
  velocidad = constrain(velocidad, 0, 255);

  if (sentido > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
  } else if (sentido < 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
  }

  analogWrite(pinPWM, velocidad);
}

// -------------------------------------------------------------------
// FUNCIÓN 2: Mover por Ángulo (Grados)
// - grados: Ángulo a desplazar (Ej: 90, 180, -90 para sentido opuesto)
// - velocidad: Velocidad PWM para realizar el movimiento
// -------------------------------------------------------------------
void moverPorAngulo(float grados, int velocidad) {
  if (grados == 0) {
    moverPorVelocidad(0, 0);
    return;
  }

  int sentido = (grados > 0) ? 1 : -1;
  float gradosAbsolutos = abs(grados);

  // Calcula el tiempo necesario en milisegundos para girar los grados indicados
  unsigned long tiempoTransicion = gradosAbsolutos * MS_POR_GRADO;

  moverPorVelocidad(velocidad, sentido);

  unsigned long tiempoInicio = millis();
  while (millis() - tiempoInicio < tiempoTransicion) {
    // Lectura constante de la corriente mientras se ejecuta el movimiento
    float corrienteActual = leerCorrienteACS712();
    Serial.print("Moviendo... Corriente ACS712: ");
    Serial.print(corrienteActual, 2);
    Serial.println(" A");
    delay(50); 
  }

  // Detener el motor al finalizar el giro
  moverPorVelocidad(0, 0);
}

void loop() {
  // --- PRUEBA DE LA FUNCIÓN 1: Control por velocidad y sentido ---
  Serial.println("Ejecutando giro por velocidad (PWM = 160, Sentido = 1)...");
  moverPorVelocidad(160, 1);
  delay(2000);

  moverPorVelocidad(0, 0); // Detener motor
  delay(1000);

  // --- PRUEBA DE LA FUNCIÓN 2: Control por grados ---
  Serial.println("Ejecutando movimiento a 90 grados a velocidad PWM 200...");
  moverPorAngulo(90.0, 200); 
  delay(2000);

  Serial.println("Ejecutando movimiento a -180 grados a velocidad PWM 200...");
  moverPorAngulo(-180.0, 200); 
  delay(3000);
}