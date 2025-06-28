#include <Wire.h>
#include <Arduino.h>
#include "AS5600.h"

// ————— Pines de conexión —————
// Sensores AS5600
#define DIR_PIN_0    4   // Pin de dirección para sensor 0
#define DIR_PIN_1    14 // Pin de dirección para sensor 1

// Motor 1 (Articulación 1)
#define ENA_PIN 27
#define IN1_PIN 26
#define IN2_PIN 25

// Motor 2 (Articulación 2)
#define ENB_PIN 33
#define IN3_PIN 32
#define IN4_PIN 15//35

// ————— Parámetros del control —————
// Parámetros PID para ambos motores
const float Kp = 2.0;              // Ganancia proporcional
const float Ki = 0.05;              // Ganancia integral
const float Kd = 0.1;              // Ganancia derivativa

const int PWM_MAX = 100;           // Máximo PWM permitido (0-255)
const float TOLERANCIA = 2.0;      // Grados de tolerancia
const unsigned long T_MUESTREO = 2; // Tiempo de muestreo (ms)

// Longitud de los eslabones en cm
const float L1 = 5.0;
const float L2 = 5.0;

// Configuración de sensores
AS5600 as5600_0(&Wire);     // Sensor para articulación 1
AS5600 as5600_1(&Wire1);   // Sensor para articulación 2

// Variables para almacenar los offsets
uint16_t offset0 = 0;
uint16_t offset1 = 0;

// Variables de control para cada motor
float anguloObjetivo0 = 0;
float anguloObjetivo1 = 0;
bool moviendose0 = false;
bool moviendose1 = false;

float errorAcumulado0 = 0.0;
float errorAcumulado1 = 0.0;
float errorAnterior0 = 0.0;
float errorAnterior1 = 0.0;

unsigned long tiempoAnterior = 0;

// Declaración de funciones (para que el compilador las vea antes de su definición)
float calculateRelativeAngle(uint16_t currentRaw, uint16_t referenceOffset);
float leerAnguloBrazo(int motor);
void moverMotor(int motor, float velocidad);
void frenarMotor(int motor);
void scanI2CBus(TwoWire &wire, const char* busName);
void controlMotor(int motor);
bool calcularAngulos(float x, float y, float& theta1, float& theta2);


// ————— Funciones auxiliares —————
// Función para calcular ángulo relativo al offset
// Función para calcular ángulo relativo al offset
float calculateRelativeAngle(uint16_t currentRaw, uint16_t referenceOffset) {
  // Offset extra deseado en grados
  const float OFFSET_GRADOS_EXTRA = -5.0;

  // Convertimos el offset extra a pasos (1 vuelta = 4096 pasos)
  int offsetExtraSteps = int((OFFSET_GRADOS_EXTRA / 360.0) * 4096.0);

  // Aplicamos offset adicional directamente sobre el offset de referencia
  int32_t difference = currentRaw - (referenceOffset + offsetExtraSteps);

  // Ajustamos la diferencia para mantenerla en el rango [0, 4095]
  if (difference < 0) difference += 4096;
  if (difference >= 4096) difference -= 4096;

  // Convertimos a grados
  return (difference * 360.0) / 4096.0;
}

// Función de lectura de ángulo
float leerAnguloBrazo(int motor) {
  if (motor == 0) {
    uint16_t raw0 = as5600_0.rawAngle(); 
    return calculateRelativeAngle(raw0, offset0);
  } else {
    uint16_t raw1 = as5600_1.rawAngle();
    return calculateRelativeAngle(raw1, offset1);
  }
}

// Función de control del motor
void moverMotor(int motor, float velocidad) {
  velocidad = constrain(velocidad, -PWM_MAX, PWM_MAX);
  int pwm = abs(velocidad);
  
  // CAMBIO SUGERIDO para Motor 0 (Articulación 1)
  if (motor == 0) {
    if (velocidad > 0) { // Antes era LOW, HIGH. Ahora lo invertimos.
      digitalWrite(IN1_PIN, HIGH); // ASUMIENDO QUE ESTA ERA LA DIRECCIÓN INVERSA
      digitalWrite(IN2_PIN, LOW);
    } else if (velocidad < 0) { // Antes era HIGH, LOW. Ahora lo invertimos.
      digitalWrite(IN1_PIN, LOW); // ASUMIENDO QUE ESTA ERA LA DIRECCIÓN CORRECTA
      digitalWrite(IN2_PIN, HIGH);
    } else {
      digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN2_PIN, LOW);
    }
    analogWrite(ENA_PIN, pwm);
  } else {
    if (velocidad > 0) { // Hacia adelante
      digitalWrite(IN3_PIN, LOW);
      digitalWrite(IN4_PIN, HIGH);
    } else if (velocidad < 0) { // Hacia atrás
      digitalWrite(IN3_PIN, HIGH);
      digitalWrite(IN4_PIN, LOW);
    } else { // Detener
      digitalWrite(IN3_PIN, LOW);
      digitalWrite(IN4_PIN, LOW);
    }
    analogWrite(ENB_PIN, pwm);
  }
}

void frenarMotor(int motor) {
  if (motor == 0) {
    digitalWrite(IN1_PIN, HIGH); // Frenado por corto a tierra
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, 0);
  } else {
    digitalWrite(IN3_PIN, HIGH); // Frenado por corto a tierra
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, 0);
  }
}

// Función para escanear dispositivos en un bus I2C
void scanI2CBus(TwoWire &wire, const char* busName) {
  byte error, address;
  int devices = 0;

  Serial.print("\nEscaneando bus ");
  Serial.println(busName);
  
  for(address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    error = wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Dispositivo encontrado en 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      devices++;
    }
    else if (error == 4) {
      Serial.print("Error desconocido en 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (devices == 0) {
    Serial.println("No se encontraron dispositivos");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\nINSTRUCCIONES:");
  Serial.println("• Ingresa coordenadas x,y en cm (ej: 5,5)");
  Serial.println("• Rango: x ∈ [-10, 10], y ∈ [0, 10]");
  Serial.println("• Presiona ENTER");
  Serial.println("---------------------------------");

  // Inicializar buses I2C
  Wire.begin();
  Wire1.begin(18, 19);   // SDA=18, SCL=19 para segundo sensor

  // Configurar pines de los motores
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  
  frenarMotor(0);
  frenarMotor(1);

  // Inicializar sensores
  // Es CRÍTICO que el brazo esté físicamente en la posición (10,0) ANTES de ejecutar el setup.
  // Esto establecerá los offsets correctamente para esa posición.
  if (as5600_0.begin(DIR_PIN_0)) {
    as5600_0.setDirection(AS5600_COUNTERCLOCK_WISE); // Ajusta según la orientación física de tu encoder y motor
    Serial.print("Sensor 0 conectado: ");
    Serial.println(as5600_0.isConnected() ? "SI" : "NO");
    delay(500);
    offset0 = as5600_0.rawAngle(); // Captura la posición raw actual como el "0 grados" del motor 0
    Serial.print("Offset sensor 0: ");
    Serial.println(offset0);
  } else {
    Serial.println("Error iniciando sensor 0");
  }

  if (as5600_1.begin(DIR_PIN_1)) {
    as5600_1.setDirection(AS5600_CLOCK_WISE); // Ajusta según la orientación física de tu encoder y motor
    Serial.print("Sensor 1 conectado: ");
    Serial.println(as5600_1.isConnected() ? "SI" : "NO");
    delay(500);
    offset1 = as5600_1.rawAngle(); // Captura la posición raw actual como el "0 grados" del motor 1
    Serial.print("Offset sensor 1: ");
    Serial.println(offset1);
  } else {
    Serial.println("Error iniciando sensor 1");
  }

  // Escaneo de buses I2C para diagnóstico
  scanI2CBus(Wire, "I2C0 (Wire)");
  scanI2CBus(Wire1, "I2C1 (Wire1)");

  Serial.print("Posicion inicial Motor 0: ");
  Serial.print(leerAnguloBrazo(0), 1);
  Serial.print(" grados\n");
  Serial.print("Posicion inicial Motor 1: ");
  Serial.print(leerAnguloBrazo(1), 1);
  Serial.println(" grados\nListo! Escribe posiciones:");
  
  tiempoAnterior = millis();
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');

    if (commaIndex > 0) {
      float x = input.substring(0, commaIndex).toFloat();
      float y = input.substring(commaIndex + 1).toFloat();
      float t1, t2;

      if (calcularAngulos(x, y, t1, t2)) {
        anguloObjetivo0 = t1;
        anguloObjetivo1 = t2;
        moviendose0 = true;
        moviendose1 = true;

        Serial.println();
        Serial.println("Posición válida.");
        Serial.print("Objetivo (x, y): ");
        Serial.print(x);
        Serial.print(", ");
        Serial.println(y);
        Serial.print("Ángulos calculados -> Motor 0: ");
        Serial.print(t1, 1);
        Serial.print("°, Motor 1: ");
        Serial.print(t2, 1);
        Serial.println("°");
      } else {
        Serial.println("ERROR: Posición fuera de alcance o fuera del rango permitido.");
      }
    } else {
      Serial.println("ERROR: Formato incorrecto. Usa: x,y");
    }
  }

  // Control PID en intervalo de muestreo
  if (millis() - tiempoAnterior >= T_MUESTREO) {
    tiempoAnterior = millis();
    
    if (moviendose0) controlMotor(0);
    if (moviendose1) controlMotor(1);
  }
}

void controlMotor(int motor) {
  float* anguloObjetivo = (motor == 0) ? &anguloObjetivo0 : &anguloObjetivo1;
  float* errorAcumulado = (motor == 0) ? &errorAcumulado0 : &errorAcumulado1;
  float* errorAnterior = (motor == 0) ? &errorAnterior0 : &errorAnterior1;
  bool* moviendose = (motor == 0) ? &moviendose0 : &moviendose1;
  
  float posicionActual = leerAnguloBrazo(motor);
  float error = *anguloObjetivo - posicionActual;

  // Normalizar error para que siempre sea el camino más corto (-180 a 180 grados)
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  if (abs(error) <= TOLERANCIA) {
    frenarMotor(motor);
    *moviendose = false;
    *errorAcumulado = 0; 
    *errorAnterior = 0; 
    Serial.print("Motor ");
    Serial.print(motor);
    Serial.print(" LISTO! Posicion: ");
    Serial.print(posicionActual, 1);
    Serial.println(" grados");
  } else {
    // Cálculo PID
    *errorAcumulado += error * (T_MUESTREO / 1000.0);
    // Limitar el error acumulado para evitar "wind-up"
    *errorAcumulado = constrain(*errorAcumulado, -200.0, 200.0);
    
    float derivada = (error - *errorAnterior) / (T_MUESTREO / 1000.0);
    *errorAnterior = error;

    float salidaPID = Kp * error + Ki * *errorAcumulado + Kd * derivada;

    // --- MODIFICACIÓN CLAVE: Aplicar un PWM mínimo ---
    // Definir un PWM mínimo para superar la inercia/fricción
    const int PWM_MIN = 70; // <--- AJUSTA ESTE VALOR. Empieza bajo (ej. 10-20) y sube gradualmente.

    if (salidaPID > 0 && salidaPID < PWM_MIN) {
      salidaPID = PWM_MIN;
    } else if (salidaPID < 0 && salidaPID > -PWM_MIN) {
      salidaPID = -PWM_MIN;
    }
    // Asegurarse de que no exceda PWM_MAX (la función moverMotor ya lo hace, pero es buena práctica)
    salidaPID = constrain(salidaPID, -PWM_MAX, PWM_MAX);

    moverMotor(motor, salidaPID);

    // Reporte periódico
    static unsigned long ultimoReporte = 0;
    if (millis() - ultimoReporte > 200) {
      ultimoReporte = millis();
      Serial.print("Motor ");
      Serial.print(motor);
      Serial.print(": Actual=");
      Serial.print(posicionActual, 1);
      Serial.print("° -> Objetivo=");
      Serial.print(*anguloObjetivo, 1);
      Serial.print("° (error=");
      Serial.print(error, 1);
      Serial.print("°, PID=");
      Serial.print(salidaPID, 1); // Agregamos la salida PID para depuración
      Serial.println("°)");
    }
  }
}

// Función de Cinemática Inversa mejorada
// Calcula los ángulos para el brazo de 2 eslabones.
// theta1: ángulo del primer eslabón respecto al eje X positivo.
// theta2: ángulo del segundo eslabón respecto al PRIMER eslabón (ángulo interno).
bool calcularAngulos(float x, float y, float& theta1, float& theta2) {
  // ... (código existente para L1, L2, distancia, cosTheta2_interno, etc.) ...

  float distancia_sq = x * x + y * y;
  float distancia = sqrt(distancia_sq);

  // 1. Verificar si el punto está dentro del alcance del brazo
  // El punto debe ser alcanzable y la 'y' debe ser no negativa
  // Añado una pequeña tolerancia para y=0 para evitar problemas de punto flotante
  if (distancia > (L1 + L2) || distancia < fabs(L1 - L2) || y < -0.001) {
    // Si 'y' es ligeramente negativa, esto puede ser un problema.
    // También, si la distancia es menor que la diferencia absoluta, es inalcanzable.
    // Si la distancia es mayor que la suma de L1+L2, también es inalcanzable.
    Serial.println("Advertencia: Posición fuera de alcance o en el cuadrante inferior (y < 0).");
    return false;
  }

  // 2. Calcular theta2 (ángulo del segundo eslabón respecto al primero)
  float cosTheta2_interno = (distancia_sq - L1 * L1 - L2 * L2) / (2 * L1 * L2);

  // Asegurar que el argumento de acos esté en el rango [-1, 1]
  if (cosTheta2_interno > 1.0) cosTheta2_interno = 1.0;
  if (cosTheta2_interno < -1.0) cosTheta2_interno = -1.0;

  // Calcula el ángulo interno del codo en radianes
  theta2 = acos(cosTheta2_interno);

  // 3. Calcular theta1 (ángulo del primer eslabón respecto al eje X)
  float alpha = atan2(y, x); // Ángulo de la línea desde el origen a (x,y)
  
  // Ángulo del triángulo entre L1, distancia, y L2
  float beta = acos((L1*L1 + distancia_sq - L2*L2) / (2 * L1 * distancia));

  // Asegurar que el argumento de acos para beta esté en el rango [-1, 1]
  if (beta > PI) beta = PI;
  if (beta < 0) beta = 0;

  // Para la solución de "codo arriba", el ángulo del primer eslabón es alpha - beta
  theta1 = alpha - beta;

  // 4. Convertir a grados
  theta1 = degrees(theta1);
  theta2 = degrees(theta2);

  // 5. Validaciones finales de los ángulos basados en las restricciones físicas del robot
  
  // Validación de theta1 (Motor 0):
  // Si tu brazo solo puede ir de 0 a 180 grados (desde el eje X positivo hasta el eje X negativo superior)
  // entonces los puntos con X negativo son válidos para theta1 > 90.
  // Tu rango de x entre -10 y 10, y entre 0 y 10, implica que theta1 puede ir de 0 a 180 grados.
  // Ajusta estos límites si tu base no permite un giro completo de 180 grados.
  if (theta1 < -1.0 || theta1 > 181.0) { // Pequeña tolerancia para los límites
    Serial.print("ERROR: Angulo del Motor 0 (Articulacion 1) fuera de rango permitido [0, 180]: ");
    Serial.println(theta1);
    return false;
  }

  // Validación de theta2 (Motor 1):
  // La restricción es +-90 grados a partir de la alineación (que es 0 grados para theta2).
  // Como Y >= 0, solo nos interesa el rango [0, 90] para theta2 (codo hacia arriba y con la limitación).
  if (theta2 < -1.0 || theta2 > 91.0) { // Pequeña tolerancia para los límites
    Serial.print("ERROR: Angulo del Motor 1 (Articulacion 2) fuera de rango permitido [0, 90]: ");
    Serial.println(theta2);
    return false;
  }
  
  return true;
}
