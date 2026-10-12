const int pinPot = A0;      // Potenciómetro (Equivalente al 34)
const int pinPWM = 9;       // Pin ENA (PWM)
const int pinIN1 = 8;       // Pin IN1 (Dirección 1)
const int pinIN2 = 7;       // Pin IN2 (Dirección 2)
const int pinEncoder = 2;   // Encoder (El pin 2 tiene interrupción de hardware en el Uno)

// --- Parámetros Mecánicos ---
// DEBES AJUSTAR ESTE VALOR: Cuántos pulsos genera tu motor para dar 1 vuelta completa del eje.
const float PULSOS_POR_REVOLUCION = 334.0; 

//Valores a insertar
float anguloObjetivo = 90.0;       // Posición a la que queremos que vaya el péndulo (Grados)
int velocidadMaxima = 150;         // Límite de velocidad máxima (PWM de 0 a 255)
float aceleracionMaxima = 2.0;     // Cuánto PWM puede cambiar por ciclo (Rampa de aceleración). Menor valor = Más suave.

//PID
float Kp = 3.5;  // Ganancia Proporcional: Fuerza de empuje hacia el objetivo
float Ki = 0.01; // Ganancia Integral: Corrige el error en estado estacionario
float Kd = 0.5;  // Ganancia Derivativa: Frena el péndulo para evitar oscilaciones (amortiguamiento)

// --- Variables de Estado del Sistema ---
volatile long contadorPulsos = 0;
int direccionActualMotor = 1; // 1 = Horario, -1 = Antihorario

float errorAcumulado = 0;
float errorAnterior = 0;
float pwmActualAplicado = 0; // Memoria para la rampa de aceleración

unsigned long tiempoAnterior = 0;
const int tiempoMuestreo = 10; // Ciclo del PID en milisegundos (100 Hz)

// =================================================================
// INTERRUPCIÓN: Lectura del Encoder
// =================================================================
void contarPulsos() {
  // Como usamos 1 solo pin para el encoder, inferimos la dirección 
  // basándonos en hacia dónde estamos mandando girar el motor.
  contadorPulsos += direccionActualMotor;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(pinPWM, OUTPUT);
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinEncoder, INPUT_PULLUP);
  
  // Asociar la interrupción al pin del encoder
  attachInterrupt(digitalPinToInterrupt(pinEncoder), contarPulsos, RISING);
  
  tiempoAnterior = millis();
  Serial.println("Iniciando Sistema de Control PID...");
}

//Enviamos señal al puente H
void accionarMotor(float salidaPWM) {
  int pwmEntero = abs((int)salidaPWM);
  
  if (salidaPWM > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
    direccionActualMotor = 1;
  } else if (salidaPWM < 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    direccionActualMotor = -1;
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
    direccionActualMotor = 0;
  }
  
  analogWrite(pinPWM, pwmEntero);
}

//PID
void computarPID() {
  unsigned long tiempoActual = millis();
  float dt = (tiempoActual - tiempoAnterior);
  
  // Ejecutar el control solo si ha pasado el tiempo de muestreo (10ms)
  if (dt >= tiempoMuestreo) {
    
    // 1. Calcular el ángulo actual en grados
    float anguloActual = ((float)contadorPulsos / PULSOS_POR_REVOLUCION) * 360.0;
    
    // 2. Calcular errores para el PID
    float error = anguloObjetivo - anguloActual;
    errorAcumulado += (error * dt);
    
    // Anti-Windup: Evita que el error integral se acumule al infinito si el motor se bloquea
    errorAcumulado = constrain(errorAcumulado, -2000, 2000);
    
    float derivadaError = (error - errorAnterior) / dt;
    
    // 3. Ecuación clásica del PID
    float salidaPID = (Kp * error) + (Ki * errorAcumulado) + (Kd * derivadaError);
    
    // 4. CONTROL DE VELOCIDAD (Clamping)
    salidaPID = constrain(salidaPID, -velocidadMaxima, velocidadMaxima);
    
    // 5. CONTROL DE ACELERACIÓN (Slew Rate Limiter)
    // Evita que el motor pase de 0 a 100 instantáneamente. Lo obliga a seguir una rampa.
    if (salidaPID > pwmActualAplicado + aceleracionMaxima) {
      pwmActualAplicado += aceleracionMaxima;
    } else if (salidaPID < pwmActualAplicado - aceleracionMaxima) {
      pwmActualAplicado -= aceleracionMaxima;
    } else {
      pwmActualAplicado = salidaPID; // Si la diferencia es menor a la aceleración, aplica el valor PID
    }
    
    // 6. Actuar sobre el hardware
    accionarMotor(pwmActualAplicado);
    
    // 7. Telemetría para graficar en el Serial Plotter
    Serial.print("Angulo_Objetivo:"); Serial.print(anguloObjetivo); Serial.print(",");
    Serial.print("Angulo_Actual:"); Serial.print(anguloActual); Serial.print(",");
    Serial.print("PWM_Suavizado:"); Serial.println(pwmActualAplicado);
    
    // 8. Preparar para el siguiente ciclo
    errorAnterior = error;
    tiempoAnterior = tiempoActual;
  }
}

void loop() {
  computarPID();
  
  // Ejemplo de cambio de Setpoint (Ángulo) dinámico en el tiempo
  // Irá de 0 a 90 grados y viceversa cada 4 segundos.
  if (millis() % 8000 < 4000) {
    anguloObjetivo = 90.0;
  } else {
    anguloObjetivo = 0.0;
  }
  
}