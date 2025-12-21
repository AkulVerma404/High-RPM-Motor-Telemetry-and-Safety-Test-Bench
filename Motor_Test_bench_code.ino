#define BLYNK_TEMPLATE_ID "enter your blynk tempelate ID"
#define BLYNK_TEMPLATE_NAME "enter your blynk template name "
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "arduinoFFT.h"
#include "HX711.h"

// =========================================
//  USER CONFIGURATION
// =========================================
char auth[] = "Blynk auth Token"; 
char ssid[] = "WiFi Name";        
char pass[] = "WiFi password";    

const float BATTERY_VOLTAGE = 12.0; 

// =========================================
//  SAFETY LIMITS
// =========================================
const float SAFETY_VIB_LIMIT_HZ = 6000.0;  
const float SAFETY_CURR_LIMIT_A = 5.0;  

// =========================================
//  PIN ASSIGNMENT
// =========================================
const int RPM_PIN = 18; 
const int AIR_SCK_PIN = 27; 
const int AIR_OUT_PIN = 14;
const int MPU_SDA_PIN = 21; 
const int MPU_SCL_PIN = 22;
const int CURRENT_PIN = 34; 
const int LC_DOUT_PIN = 25;
const int LC_SCK_PIN = 26;
const int RELAY_PIN = 4; 

// =========================================
//  GLOBAL VARIABLES
// =========================================
BlynkTimer timer; 

// --- RPM ---
const int blades = 2;
const unsigned long debounceMicros = 2000;
const unsigned long timeoutValue = 1000000;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulseAvailable = false;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- Air Pressure ---
float PRES_SCALE_FACTOR = 450000.0;
float PRES_DEADZONE = 0.15;
const float AIR_DENSITY = 1.225;
long PRES_OFFSET = 0;

// --- Vibration ---
#define SAMPLES 128
#define SAMPLING_FREQUENCY 512 
ArduinoFFT<double> FFT = ArduinoFFT<double>();
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
Adafruit_MPU6050 mpu;
bool mpu_active = false; 

// --- Current ---
const float CURR_SENSITIVITY = 0.066; 
const float ADC_VOLTAGE = 3.3;    
float curr_zeroPoint = 2048; 
float curr_filteredAdc = 0;

// --- Load Cell ---
HX711 scale;
float LC_CALIBRATION_FACTOR = 400.0; 
const int SMOOTHING_WINDOW = 10;
float lc_readings[SMOOTHING_WINDOW];
int lc_readIndex = 0;
float lc_total = 0;
float lc_average = 0;

// --- Blynk Data Variables ---
float blynk_weight = 0;    
int   blynk_rpm = 0;       
float blynk_vib = 0;       
float blynk_current = 0;   
float blynk_pressure = 0;  
float blynk_eff = 0;       
int   blynk_estop = 0;     

// --- Interrupt ---
void IRAM_ATTR bladeDetected() {
  unsigned long currentTime = micros();
  unsigned long rawInterval = currentTime - lastPulseTime;
  if (rawInterval > debounceMicros) {
    pulseInterval = rawInterval;
    lastPulseTime = currentTime;
    newPulseAvailable = true;
  }
}

// --- Helper: Pressure Read with Timeout ---
long readPressureRaw() {
    unsigned long result = 0;
    unsigned long startTime = millis();
    while (digitalRead(AIR_OUT_PIN) == HIGH) {
      if (millis() - startTime > 50) return 0; // Timeout
    }
    for (int i = 0; i < 24; i++) {
      digitalWrite(AIR_SCK_PIN, HIGH); delayMicroseconds(1);
      digitalWrite(AIR_SCK_PIN, LOW);  delayMicroseconds(1);
      result = result << 1;
      if (digitalRead(AIR_OUT_PIN) == HIGH) result++;
    }
    digitalWrite(AIR_SCK_PIN, HIGH); delayMicroseconds(1);
    digitalWrite(AIR_SCK_PIN, LOW);
    long finalValue = result;
    if (result & 0x800000) finalValue |= 0xFF000000;
    return finalValue;
}

// =========================================
//  BLYNK SENDER
// =========================================
void sendSensorData() {
  Blynk.virtualWrite(V0, blynk_weight);   
  Blynk.virtualWrite(V1, blynk_rpm);      
  Blynk.virtualWrite(V2, blynk_vib);      
  Blynk.virtualWrite(V3, blynk_current);  
  Blynk.virtualWrite(V4, blynk_pressure); 
  Blynk.virtualWrite(V5, blynk_eff);      
  Blynk.virtualWrite(V6, blynk_estop);    
}

// =========================================
//  AUTO-RETRY SENSORS
// =========================================
void checkSensors() {
  // If Vibration sensor is dead, try to restart it
  if (!mpu_active) {
    if (mpu.begin()) {
      mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
      mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
      mpu_active = true;
      Serial.println("[SYSTEM] MPU6050 Reconnected!");
    }
  }
}

// =========================================
//  SETUP
// =========================================
void setup() {
  Serial.begin(115200);
  
  Serial.println("Connecting to Blynk...");
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, sendSensorData);
  timer.setInterval(5000L, checkSensors); // Retry dead sensors every 5s

  Serial.println("\n=== JET TURBINE ANALYZER + IOT STARTED ===");

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); 

  pinMode(RPM_PIN, INPUT); 
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), bladeDetected, FALLING);

  pinMode(AIR_SCK_PIN, OUTPUT);
  pinMode(AIR_OUT_PIN, INPUT_PULLUP);
  digitalWrite(AIR_SCK_PIN, HIGH); delayMicroseconds(100); digitalWrite(AIR_SCK_PIN, LOW);
  
  long p_sum = 0;
  for (int i = 0; i < 10; i++) { 
    long val = readPressureRaw();
    if (val != 0) p_sum += val; 
    delay(20); 
  }
  PRES_OFFSET = p_sum / 10;

  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000);
  if (!mpu.begin()) { 
    Serial.println("[ERROR] MPU6050 Not Found! Will retry later...");
    mpu_active = false;
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    mpu_active = true;
  }
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

  pinMode(CURRENT_PIN, INPUT);
  long c_sum = 0;
  for(int i=0; i<1000; i++) { c_sum += analogRead(CURRENT_PIN); delayMicroseconds(50); }
  curr_zeroPoint = c_sum / 1000.0;
  curr_filteredAdc = curr_zeroPoint;

  for (int i = 0; i < SMOOTHING_WINDOW; i++) lc_readings[i] = 0;
  scale.begin(LC_DOUT_PIN, LC_SCK_PIN);
  
  if (scale.is_ready()) {
    scale.set_scale(LC_CALIBRATION_FACTOR); 
    scale.tare(); 
  } else { 
    Serial.println("[ERROR] HX711 Not Found!"); 
  }

  Serial.println("=== SYSTEM READY ===");
}

// =========================================
//  MAIN LOOP
// =========================================
void loop() {
  Blynk.run();
  timer.run();

  // 1. VIBRATION (Skipped if sensor missing)
  double peakFrequency = 0;
  if (mpu_active) {
    for(int i=0; i<SAMPLES; i++) {
        microseconds = micros();
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        double totalAccel = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));
        vReal[i] = totalAccel - 9.81;
        vImag[i] = 0; 
        while(micros() - microseconds < sampling_period_us){ }
    }
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);
    peakFrequency = FFT.majorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    if (peakFrequency < 19.0) peakFrequency = 0.0;
  }

  // 2. RPM
  unsigned int rpm = 0;
  static bool rpm_stopped = false;
  unsigned long intervalCopy = 0;
  bool dataReady = false;
  unsigned long lastTimeCopy = 0;
  portENTER_CRITICAL(&timerMux);
  if (newPulseAvailable) { intervalCopy = pulseInterval; newPulseAvailable = false; dataReady = true; }
  lastTimeCopy = lastPulseTime;
  portEXIT_CRITICAL(&timerMux);
  if ((micros() - lastTimeCopy) > timeoutValue) { rpm_stopped = true; rpm = 0; }
  else if (dataReady && intervalCopy > 0) { 
    rpm = round((60000000.0 / intervalCopy) / blades);
  }

  // 3. AIR PRESSURE
  float kPa = 0;
  float velocity = 0;
  long rawValue = readPressureRaw(); 
  if (rawValue != 0) {
    kPa = (rawValue - PRES_OFFSET) / PRES_SCALE_FACTOR;
    if (kPa <= PRES_DEADZONE) kPa = 0.00;
    if (kPa > 0) velocity = sqrt((2 * (kPa * 1000.0)) / AIR_DENSITY);
  }

  // 4. CURRENT
  int rawAdc = analogRead(CURRENT_PIN);
  float alpha = 0.05;
  curr_filteredAdc = (alpha * rawAdc) + ((1.0 - alpha) * curr_filteredAdc);
  float voltage = (curr_filteredAdc / 4095.0) * ADC_VOLTAGE;
  float zeroVolts = (curr_zeroPoint / 4095.0) * ADC_VOLTAGE;
  float current_amps = (voltage - zeroVolts) / CURR_SENSITIVITY;
  if (current_amps < 0.30) current_amps = 0.0;

  // 5. LOAD CELL
  float load_weight_g = 0;
  if (scale.is_ready()) {
    float raw_mass = scale.get_units(1); 
    lc_total = lc_total - lc_readings[lc_readIndex];
    lc_readings[lc_readIndex] = raw_mass;
    lc_total = lc_total + lc_readings[lc_readIndex];
    lc_readIndex = lc_readIndex + 1;
    if (lc_readIndex >= SMOOTHING_WINDOW) lc_readIndex = 0;
    load_weight_g = lc_total / SMOOTHING_WINDOW;
    if (load_weight_g < 0) load_weight_g = 0.0;
  }

  // 6. EFFICIENCY
  float efficiency_percent = 0.0;
  float thrust_newtons = load_weight_g * 0.00981; 
  float numerator = thrust_newtons * velocity;     
  float denominator = BATTERY_VOLTAGE * current_amps; 
  if (denominator > 1.0) { efficiency_percent = (numerator / denominator) * 100.0; }

  // 7. SAFETY
  bool danger = (peakFrequency > SAFETY_VIB_LIMIT_HZ) || (current_amps > SAFETY_CURR_LIMIT_A);
  digitalWrite(RELAY_PIN, danger ? HIGH : LOW);
  
  blynk_weight = load_weight_g;
  blynk_rpm = rpm;
  blynk_vib = peakFrequency;
  blynk_current = current_amps;
  blynk_pressure = kPa;           
  blynk_eff = efficiency_percent;
  blynk_estop = danger ? 1 : 0;   
  
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("RPM:"); Serial.print(rpm);
    Serial.print(" | Wt:"); Serial.print(load_weight_g, 0);
    Serial.print(" | Pres:"); Serial.print(kPa, 2);
    Serial.print(" | Vib:"); Serial.print(peakFrequency, 0);
    Serial.print(" | Curr:"); Serial.print(current_amps, 2);
    Serial.print(" | Eff%:"); Serial.println(efficiency_percent, 1);
  }
}
