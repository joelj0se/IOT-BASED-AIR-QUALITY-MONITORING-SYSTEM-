// =========================================================================
// FINAL AIR QUALITY MONITOR (MQ-135 + DHT11) WITH BLYNK 2.0 AND BUZZER ALARM
// NodeMCU ESP8266
// =========================================================================

#define BLYNK_PRINT Serial

// ** BLYNK TEMPLATE DEFINITIONS - REQUIRED FOR BLYNK 2.0 **
#define BLYNK_TEMPLATE_ID   "TMPL3ksMm7_yz" 
#define BLYNK_TEMPLATE_NAME "AIR QUALITY MONITORING SYSTEM"

// Libraries
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// --- USER CONFIGURATION REQUIRED ---
char auth[] = "14Fk01w4U5drBDWfWo3AoSdYgduqIo39"; // Your Auth Token
char ssid[] = "Moolamkunnam";                         // Your NEW WiFi SSID 
char pass[] = "44jj00oo";                      // Your NEW WiFi password 
// -----------------------------------

// --- HARDWARE PIN DEFINITIONS ---
const int GAS_SENSOR_PIN = A0;   // MQ-135 Analog Output
const int DHT_PIN = D4;          // DHT11 Data pin (GPIO2)
const int BUZZER_PIN = D0;       // Buzzer Control pin (GPIO16)
const int ALARM_LED_PIN = D6;    // Physical LED Pin (GPIO12)
#define DHTTYPE DHT11

// --- GAS SENSOR CALIBRATION AND CONSTANTS ---
// R0 is the CALIBRATED Clean Air Resistance from Phase 4.1. This is a placeholder value.
// *** YOUR CALIBRATED VALUE ***
const float R0_CLEAN_AIR = 258578.80; 

// Volatile Organic Compound (VOC) Alarm Threshold (in PPM equivalent)
// Set to a low value now that we expect the buzzer to be controllable.
const float ALARM_THRESHOLD_PPM = 250.0; // <<< CHANGED FROM 1500.0 TO 500.0

// MQ-135 Hardware Constants for Rs calculation (based on 2.2k + 1k divider)
const float R_DIVIDER_TOP = 2200.0; // **FIXED**: Using a single 2.2k resistor
const float R_DIVIDER_BOTTOM = 1000.0; 
const float V_CC = 5.0; 
const float R_L_MODULE = 10000.0; 

// MQ-135 CO2 Curve Constants (A * (Rs/R0)^B)
const float CO2_A = 116.603; 
const float CO2_B = -2.769;  

// --- GLOBAL OBJECTS ---
DHT dht(DHT_PIN, DHTTYPE);
BlynkTimer timer;

// --- FUNCTIONS ---

/**
 * @brief Calculates the Sensor Resistance (Rs) from the raw A0 reading.
 */
float calculateRs(int raw_adc) {
  // 1. Calculate the scaled voltage (V_A0) at the A0 pin (NodeMCU max A0 is 1.0V)
  float V_A0 = (float)raw_adc * (1.0 / 1024.0);

  // 2. Calculate the True Sensor Output Voltage (V_out) before the divider
  float V_out = V_A0 * (R_DIVIDER_TOP + R_DIVIDER_BOTTOM) / R_DIVIDER_BOTTOM;

  // 3. Calculate Sensor Resistance (Rs)
  float Rs = (V_CC / V_out - 1.0) * R_L_MODULE;
  
  return Rs;
}

/**
 * @brief Calculates the environment compensation factor based on T and H.
 */
float getCorrectionFactor(float t, float h) {
  if (t < 20) t = 20;
  if (t > 40) t = 40;
  if (h < 30) h = 30;
  if (h > 80) h = 80;

  float corr = 1.0;
  float t_corr = 1.0 + 0.0123 * (t - 20.0);
  float h_corr = 1.0 + 0.015 * (h - 50.0);
  
  corr = t_corr * h_corr;
  return corr;
}

/**
 * @brief Calculates the PPM using the power-law curve.
 */
float calculatePPM(float Rs_R0_ratio) {
  return CO2_A * pow(Rs_R0_ratio, CO2_B);
}

/**
 * @brief Controls the buzzer (now using tone/noTone for 2-pin passive buzzer).
 * The 2-pin passive buzzer requires a tone (frequency) to sound.
 */
void buzz(bool shouldBuzz) {
  if (shouldBuzz) {
    // 800 Hz tone is generated on the pin (adjust frequency for sound)
    tone(BUZZER_PIN, 800); 
  } else {
    // Stops the tone generation (silences the buzzer)
    noTone(BUZZER_PIN); 
  }
}

/**
 * @brief Reads sensors, calculates data, and sends it to Blynk.
 */
void sendSensorData() {
  
  // 1. Read DHT11
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  // Check if any reads failed (DHT read sometimes fails)
  if (isnan(h) || isnan(t)) {
    Serial.println("ERROR: Failed to read from DHT sensor! Skipping this loop.");
    return;
  }

  // 2. Read MQ-135 (Raw)
  int raw_adc = analogRead(GAS_SENSOR_PIN);
  float Rs = calculateRs(raw_adc);
  
  // 3. Apply Compensation to R0
  float correction_factor = getCorrectionFactor(t, h);
  float R0_compensated = R0_CLEAN_AIR / correction_factor;
  
  // 4. Calculate the Rs/R0 ratio and Gas Concentration (PPM)
  float Rs_R0_ratio = Rs / R0_compensated;
  float ppm = calculatePPM(Rs_R0_ratio);
  
  // 5. Send data to Blynk Virtual Pins
  Blynk.virtualWrite(V1, t);   // V1: Temperature (C)
  Blynk.virtualWrite(V2, h);   // V2: Humidity (%)
  Blynk.virtualWrite(V3, ppm); // V3: Air Quality (PPM)

  // 6. Check for Alarm Condition
  if (ppm > ALARM_THRESHOLD_PPM) {
    // Alarm is ON
    buzz(true); // *** ACTIVATE BUZZER via tone() ***
    digitalWrite(ALARM_LED_PIN, HIGH); // Turn Physical LED ON
    Blynk.virtualWrite(V4, 1); // V4: Send 1 (ON) for Virtual LED (RED)
    Serial.printf("!!! ALERT: PPM %.0f exceeded threshold (%.0f) !!!\n", ppm, ALARM_THRESHOLD_PPM);
  } else {
    // Alarm is OFF
    buzz(false); // *** DEACTIVATE BUZZER via noTone() ***
    digitalWrite(ALARM_LED_PIN, LOW); // Turn Physical LED OFF
    Blynk.virtualWrite(V4, 0); // V4: Send 0 (OFF) for Virtual LED (GREEN)
  }

  // 7. Print to Serial Monitor
  Serial.printf("T: %.1f C, H: %.1f %%, PPM: %.0f\n", t, h, ppm);
}


// ------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100); 
  Serial.println("\n--- Starting Full Air Quality Monitor ---");
  
  // 1. Initialize buzzer and LED pins
  pinMode(BUZZER_PIN, OUTPUT);
  // *** Initial state must be silenced using noTone() for passive buzzer ***
  noTone(BUZZER_PIN); 

  pinMode(ALARM_LED_PIN, OUTPUT); // New LED Pin Initialization
  digitalWrite(ALARM_LED_PIN, LOW); // LED OFF

  dht.begin();
  
  // 2. Attempt to connect to Wi-Fi
  Serial.printf("Connecting to Wi-Fi: %s\n", ssid);
  WiFi.begin(ssid, pass);

  // Wait for Wi-Fi to connect with a 30 second timeout
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 6) {
    delay(5000); // Wait 5 seconds
    Serial.print(".");
    attempt++;
  }

  // 3. Check Wi-Fi Result
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi Connected!");
    Serial.printf("Attempting to connect to Blynk Server with Auth Token: %s\n", auth);
    
    // Connect to Blynk (this usually connects fast if Wi-Fi is up)
    Blynk.config(auth);
    Blynk.connect();
    
    if (Blynk.connected()) {
      Serial.println("--- Wi-Fi and Blynk Connected! ---");
      Serial.println("--- Starting Sensor Timer ---");
      // 4. Setup Timer to run sensor reading every 5 seconds (5000ms)
      timer.setInterval(5000L, sendSensorData);
    } else {
      // Failed to connect to Blynk server even with Wi-Fi up
      Serial.println("!!! FAILED TO CONNECT TO BLYNK SERVER (Auth Token or Server Issue) !!!");
    }
  } else {
    // Failed to connect to Wi-Fi network
    Serial.printf("\n!!! FAILED TO CONNECT TO Wi-Fi '%s' (30 sec timeout reached) !!!\n", ssid);
    Serial.println("Check SSID/Password, or $2.4\text{GHz}$ band setting on hotspot.");
  }
}

// ------------------------------------------------------------------
// LOOP
// ------------------------------------------------------------------
void loop() {
  // Run Blynk and Timer ONLY if Blynk is connected.
  if (Blynk.connected()) {
    Blynk.run();
    timer.run();
  }
  // If not connected, the board sits here silently (not ideal, but safe)
}
