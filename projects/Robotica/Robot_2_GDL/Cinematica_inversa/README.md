# Robótica – Cinemática Directa e Inversa para Brazo de 2 Grados de Libertad

Este proyecto implementa el control de posición angular de un brazo robótico de 2 grados de libertad usando dos motores DC, dos encoders magnéticos AS5600 y un controlador PID. Permite alternar entre cinemática directa (calcular posición cartesiana a partir de ángulos) y cinemática inversa (calcular ángulos necesarios para alcanzar una coordenada X,Y), todo vía monitor serie.
![deepseek_mermaid_20250701_7d3c3b](https://github.com/user-attachments/assets/3bde23ec-0587-4a02-9a0d-a4544220e5c3)

## 🎯 Objetivo

1. **Cinemática directa**: ingresar dos ángulos (θ₁ y θ₂) y calcular la posición (X, Y) de la punta del brazo.  
2. **Cinemática inversa**: ingresar una coordenada (X, Y) en cm y calcular los ángulos (θ₁, θ₂) para que la punta del brazo alcance dicha posición.  
3. Ejecutar el movimiento mediante control PID para cada motor, con tolerancia de ±2° y velocidad limitada.  
4. Ofrecer robustez: filtros de entrada, verificación de sensores y límites físicos (ángulos entre 0° y 180 °, coordenadas en [0, 10] cm).

## ⚙️ Hardware utilizado

- 2 × Motor DC con reductora  
- 2 × Encoder magnético AS5600 (uno en cada articulación)  
- Puente H L298 (canal A para motor 1, canal B para motor 2)  
- ESP32 DOVKIT V1  
- Fuente externa de 12 V  
- Cables para I²C y pines de control de velocidad/dirección (ENA, IN1, IN2; ENB, IN3, IN4)

## 📐 Parámetros mecánicos y eléctricos

- Longitud de eslabón 1 (L1): 5.0 cm  
- Longitud de eslabón 2 (L2): 5.0 cm  
- PWM máximo: 0–100 (mapeado internamente a 0–255)  
- Tolerancia angular: ±2.0°  
- Tiempo de muestreo del lazo de control: 2 ms  

## 🧠 Principales características del sistema

### 1. Selección de modo por monitor serie

- Al iniciar, el sistema escanea ambos buses I²C (Wire y Wire1) y muestra direcciones encontradas.  
- Luego imprime el menú:
  
Seleccione modo de operación:
1. Cinemática directa (ingrese ángulos)
2. Cinemática inversa (ingrese coordenadas x,y)


### 2. Cinemática directa

- **Entrada**: dos ángulos en grados separados por coma (ej: `90,45`).  
- **Cálculo**:  
- X = L1·cos(θ₁) + L2·cos(θ₁ + θ₂)  
- Y = L1·sin(θ₁) + L2·sin(θ₁ + θ₂)  
- **Salida**: muestra X y Y (en cm) y luego ejecuta el movimiento si se desea “mover” a dicha posición calculada.

### 3. Cinemática inversa

- **Entrada**: coordenadas X,Y en cm separadas por coma (ej: `5,5`), rango válido X ∈ [0,10], Y ∈ [0,10].  
- **Cálculo** (ley de cosenos y funciones trigonométricas):  
- cos(θ₂) = (X² + Y² − L1² − L2²) / (2·L1·L2)  
- θ₂ = acos(...)  
- θ₁ = atan2(Y, X) − atan2(L2·sin(θ₂), L1 + L2·cos(θ₂))  
- **Verificación**: comprueba si la posición está dentro del alcance físico y en cuadrante válido (Y ≥ 0).  
- **Acción**: si es válida, asigna los ángulos objetivo y arranca motores.

### 4. Control PID para cada motor

- Ganancias:  
- Kp = 2.0  
- Ki = 0.05  
- Kd = 0.10  
- Se calcula error = θ_objetivo − θ_actual  
- Se integra y deriva error; salida u = Kp·error + Ki·∑error·Δt + Kd·(Δerror/Δt)  
- PWM saturado a ±100 (mapeado)  
- Se invierte dirección según el signo de u  
- Freno automático al alcanzar el objetivo dentro de tolerancia

### 5. Robustez y seguridad

- Conversión de RAW (0–4095) a grados (0–360) con corrección de discontinuidades.  
- Offset inicial tomado al encender y ajustado con un desplazamiento extra de −5°.  
- Verificación de conexión de ambos AS5600 antes de empezar.  
- Filtro de “rebote” en entrada serial para evitar lecturas erráticas.  
- Límites físicos: ángulos entre 0° y 180°, coordenadas en el semiplano Y ≥ 0.

## 📁 Estructura del código

- **main.ino**  
- `setup()`  
  - Inicializa Serial (115200 baudios), Wire, Wire1  
  - Escanea I²C y muestra direcciones  
  - Configura offsets de encoders  
  - Imprime menú inicial  
- `loop()`  
  - Si espera comando, lee línea serial y determina modo  
  - En modo directa/inversa: parsea valores, valida rango, calcula objetivos  
  - Activa flags `moviendose0` y `moviendose1`  
  - Llama a `controlMotor(0)` y `controlMotor(1)` cada 2 ms  
- **Funciones auxiliares**  
- `calculateRelativeAngle(raw, offset)` → convierte RAW a grados relativos  
- `leerAnguloBrazo(motor)` → obtiene ángulo actual ajustado  
- `moverMotor(motor, velocidad)` → setea PWM y dirección  
- `frenarMotor(motor)` → detiene motor  
- `scanI2CBus(wire, busName)` → imprime dispositivos encontrados  
- `calcularAngulos(x, y, &theta1, &theta2)` → resuelve cinemática inversa  

## 🖥️ Cómo usar

1. Cargar `Cinematica_inversa_v4_y_cinematica_directa.ino` en el ESP32.  
2. Abrir Monitor Serie a 115200 baudios.  
3. Esperar a que finalice el escaneo I²C y aparezca el menú.  
4. Ingresar `1` + ENTER para cinemática directa y escribir “θ1,θ2” (grados).  
5. Ingresar `2` + ENTER para cinemática inversa y escribir “x,y” (cm).  
6. El sistema mostrará cálculos y moverá los motores hasta la posición deseada.  

## 📸 Capturas y Funcionamiento

![Captura del proyecto](screenshot.png)

<video src="https://github.com/user-attachments/assets/4a18f0a9-47c6-4eb9-9952-6b54f74541a8.mp4" controls style="max-width:100%;"></video>
<video src="https://github.com/user-attachments/assets/fbe9209d-be4b-495e-834e-1509b448d2aa.mp4" controls style="max-width:100%;"></video>
<video src="https://github.com/user-attachments/assets/9b6261e0-5f73-4eb6-aaa3-88af923ea95c.mp4" controls style="max-width:100%;"></video>


## 📁 Archivos incluidos

- `main.ino`: código completo del sistema
- `screenshot.png`: imagen de referencia visual del proyecto


---

Desarrollado como entregable para la materia **Fundamentos de Robótica**.  
