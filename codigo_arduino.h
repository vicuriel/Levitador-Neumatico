#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X tof;

//Pines Driver 
const int pinENA = 25;   // PWM
const int pinIN3 = 27;   // Dirección
const int pinIN4 = 26;   // Dirección

//Escala y límites físicos 
const float d_cm_min = 4.0f;     // pelota casi tocando el sensor
const float d_cm_max = 73.4f;    // fondo: lectura más grande
const float altura_min = 0.0f;
const float altura_max = (d_cm_max - d_cm_min);  

// Ventilador 24V: zona muerta ~22.4V
const int PWM_RAW_MAX = 255;
const int PWM_MIN_EFECTIVO = 215;   // (22.4/24)*255 ≈ 239-242, ajustado
const int PWM_MAX_EFECTIVO = 250;   // pequeño margen
bool enable_zero_cutoff = true;     // permitir 0 cuando ref=0

//PID
volatile float Kp = 4.92f;
volatile float Ki = 2.5f;
volatile float Kd = 2.25f;

//Tiempos
volatile uint16_t Ts_ms = 20;  // período de muestreo (ms)
unsigned long t_prev = 0;

//Estado PID
float ref_altura_cm = 20.0f;  // referencia inicial (0..altura_max)
float e = 0.0f, e_prev = 0.0f, I = 0.0f;
float y_altura = 0.0f;

//Derivada filtrada (filtro 1er orden)
float der = 0.0f;
const float alpha = 0.2f; 

// Salida PWM
int pwm_cmd = 0;

// Flags
bool refSetFromSerial = false;

// Utils
float clampf(float v, float a, float b){
  return (v < a) ? a : (v > b ? b : v);
}

void setFanPWM(int pwm){
  pwm = constrain(pwm, 0, PWM_RAW_MAX);

  digitalWrite(pinIN4, HIGH);
  digitalWrite(pinIN3, LOW);

  analogWrite(pinENA, pwm);
}

float readAlturaCM(){
  // Lee VL53L0X en mm
  uint16_t mm = tof.readRangeSingleMillimeters();
  if (tof.timeoutOccurred()) {
    return NAN;
  }
  float d_cm = mm / 10.0f;

  // Clip a [d_cm_min, d_cm_max] cm por seguridad
  d_cm = clampf(d_cm, d_cm_min, d_cm_max);

  // Mapeo: altura = d_max - d  
  float altura = d_cm_max - d_cm;

  // Clip a [0, altura_max]
  altura = clampf(altura, altura_min, altura_max);
  return altura;
}

void processCommand(String command) {
  command.trim();
  if (command.length() < 3) return;
  char param = command.charAt(0);
  float value = command.substring(2).toFloat();

  switch (param) {
    case 'p': 
      Kp = value; 
      Serial.print("Kp="); Serial.println(Kp); 
      break;
    case 'i': 
      Ki = value; 
      Serial.print("Ki="); Serial.println(Ki); 
      break;
    case 'd': 
      Kd = value; 
      Serial.print("Kd="); Serial.println(Kd); 
      break;
    case 'r':
      // referencia válida 0 a altura_max
      ref_altura_cm = clampf(value, altura_min, altura_max);
      refSetFromSerial = true;
      Serial.print("Ref_altura_cm="); Serial.println(ref_altura_cm);
      break;
    case 't':
      Ts_ms = (uint16_t) max(5.0f, value);
      Serial.print("Ts_ms="); Serial.println(Ts_ms);
      break;
    case 'z':
      enable_zero_cutoff = (value > 0.5f);
      Serial.print("ZeroCutoff="); Serial.println(enable_zero_cutoff ? "ON" : "OFF");
      break;
    default:
      Serial.print("Comandos: p/i/d <val>, r <cm 0..");
      Serial.print(altura_max, 1);
      Serial.println(">, t <Ts ms>, z <0/1>");
      break;
  }
}

void setup(){
  Serial.begin(115200);
  Wire.begin();

  // TOF
  tof.setTimeout(50);
  if(!tof.init()){
    Serial.println("VL53L0X no encontrado!");
    // Seguimos pero sin control
  }
  tof.setMeasurementTimingBudget(20000); // ~20ms

  // Driver
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENA, OUTPUT);

  // Dirección fija (adelante)
  digitalWrite(pinIN4, HIGH);
  digitalWrite(pinIN3, LOW);

  // Ramp-up de inicio para vencer inercia/flujo
  setFanPWM(PWM_MIN_EFECTIVO);
  delay(500);

  t_prev = millis();
}

void loop(){
  // Comandos por Serial
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  unsigned long t_now = millis();
  if ((t_now - t_prev) < Ts_ms) return; // muestreo fijo
  float Ts = (t_now - t_prev)/1000.0f;
  t_prev = t_now;

  // Medición del sensor
  float y = readAlturaCM();
  if (isnan(y)){
    // Sensor falló (apaga o mantiene mínimo)
    if (enable_zero_cutoff) setFanPWM(0);
    else setFanPWM(PWM_MIN_EFECTIVO);
    Serial.println("Sensor timeout");
    return;
  }
  y_altura = y;

  // Error (en cm)
  e = ref_altura_cm - y_altura;

  // Derivada filtrada del error (1er orden)
  float de = (e - e_prev)/Ts;
  der = (1.0f - alpha)*der + alpha*de;

  // PID "en posición" con ANTI-WINDUP
  // Vista previa de la salida con el integrador actual
  float P = Kp * e;
  float D = Kd * der;
  float u_pid_preview = P + I + D;

  // Mapeo preliminar a PWM para detectar saturación
  int pwm_preview;
  if (enable_zero_cutoff && ref_altura_cm <= 0.1f) {
    pwm_preview = 0;
  } else {
    float u_norm = clampf(u_pid_preview / 40.0f, 0.0f, 1.0f);
    int span = (PWM_MAX_EFECTIVO - PWM_MIN_EFECTIVO);
    pwm_preview = PWM_MIN_EFECTIVO + (int)(u_norm * span);
  }

  bool satur_hi = (pwm_preview >= PWM_MAX_EFECTIVO);
  bool satur_lo = (enable_zero_cutoff ? (pwm_preview <= 0)
                                      : (pwm_preview <= PWM_MIN_EFECTIVO));

  // Regla anti-windup:
  // - Si estoy saturado arriba y el error quiere empujar más (e>0) -> no integro
  // - Si estoy saturado abajo y el error quiere empujar más hacia abajo (e<0) -> no integro
  bool allow_I = true;
  if (satur_hi && (e > 0)) allow_I = false;
  if (satur_lo && (e < 0)) allow_I = false;

  if (allow_I){
    I += Ki * e * Ts;
    I = clampf(I, -100.0f, 100.0f);   // límite suave del integrador
  }

  // Recalcular salida ya con I corregio
  float u_pid = Kp*e + I + Kd*der;

  int pwm_out;
  if (enable_zero_cutoff && ref_altura_cm <= 0.1f) {
    pwm_out = 0; // referencia 0 => apagado total
  } else {
    float u_norm = clampf(u_pid / 40.0f, 0.0f, 1.0f);
    int span = (PWM_MAX_EFECTIVO - PWM_MIN_EFECTIVO);
    pwm_out = PWM_MIN_EFECTIVO + (int)(u_norm * span);
  }

  pwm_cmd = constrain(pwm_out, 0, PWM_RAW_MAX);
  setFanPWM(pwm_cmd);

  // Imprimimos los resultados
  Serial.print("Ref_cm:");    Serial.print(ref_altura_cm);
  Serial.print("\tAlt_cm:");  Serial.print(y_altura);
  Serial.print("\tErr:");     Serial.print(e);
  Serial.print("\tPWM:");     Serial.print(pwm_cmd);
  Serial.print("\tp:");       Serial.print(Kp);
  Serial.print("\ti:");       Serial.print(Ki);
  Serial.print("\td:");       Serial.println(Kd);

  e_prev = e;
}


/*
IMPORTANTE

- b: parámetro aerodinámico (1/s)  
- kv: ganancia del ventilador (m/s por PWM)  
- τ: constante de tiempo del ventilador (s)  
- s: dinámica inercial (integrador)  
- s + b: polo aerodinámico  
- τs + 1: ventilador como primer orden  
- e^{-T_d s}: retardo total del sistema  
- Y(s): altura (cm)  
- U(s): señal PWM

*/


