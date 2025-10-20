🏡 IoT Air Quality & Environmental Monitor (NodeMCU ESP8266 + Blynk)

Project Overview

This project develops a complete Internet of Things (IoT) monitoring system using the NodeMCU ESP8266 microcontroller. It tracks key environmental metrics—Gas Concentration (Air Quality in $\text{PPM}$ equivalent), Temperature, and Humidity—and sends the data in real-time to the Blynk $2.0$ platform for remote viewing and alerts.

The system features a two-tiered alarm system to provide both immediate physical alerts and remote digital notifications:

Physical Alarm: Activates a Buzzer and Physical LED for immediate local alerts at a low threshold ($\mathbf{50}$ $\text{PPM}$).

Virtual Alarm: Activates a Blynk Virtual LED for remote/dashboard notification at a higher, more critical threshold ($\mathbf{250}$ $\text{PPM}$).

🛠️ Hardware & Components

Component

Purpose

Pin Connection (NodeMCU)

NodeMCU ESP8266

Microcontroller, Wi-Fi connectivity

N/A

MQ-135 Gas Sensor

Monitors Air Quality ($\text{PPM}$ equivalent)

Analog Pin A0

DHT11 Sensor

Measures Temperature and Humidity

Digital Pin D4 (GPIO2)

2-Pin Passive Buzzer

Local Auditory Alarm

Digital Pin D0 (GPIO16)

LED

Local Visual Alarm

Digital Pin D6 (GPIO12)

Resistor

Current limiting for Buzzer

$1\text{k}\Omega$ (in series with $\text{GND}$)

Resistor

Current limiting for LED

$220\Omega$ (in series with $\text{GND}$)

🔌 Wiring Diagram

The circuit ensures safe operation for the output devices using current-limiting resistors, which was crucial for controlling the passive buzzer.

Connection Summary

Component Pin

NodeMCU Connection

Notes

MQ-135 $\text{V}_{\text{CC}}$

$\mathbf{5\text{V}}$ or $\mathbf{V}_{\text{in}}$

Powers the heater element.

MQ-135 $\text{A}_{\text{OUT}}$

$\mathbf{A0}$

Analog Signal Input.

DHT11 $\text{V}_{\text{CC}}$

$\mathbf{3.3\text{V}}$

Stable power for the $\text{DHT}11$.

DHT11 Data

$\mathbf{D4}$

Digital Signal Input.

Buzzer Pin (+)

$\mathbf{D0}$

Controlled by the tone() function.

Buzzer Pin (-)

$\mathbf{1\text{k}\Omega}$ Resistor $\rightarrow$ GND

Critical for loudness and safety.

Alarm LED (+) Anode

$\mathbf{D6}$

Physical alarm indicator.

Alarm LED (-) Cathode

$\mathbf{220\Omega}$ Resistor $\rightarrow$ GND

Required for LED protection.

All GND Pins

$\mathbf{GND}$

Common Ground.

💻 Code and Setup

The project uses the Arduino IDE for programming the NodeMCU.

Prerequisites (Libraries)

Ensure the following libraries are installed via the Arduino Library Manager:

Blynk by Volodymyr Shymanskyy (Version $1.2.2$ or later)

DHT sensor library by Adafruit

Adafruit Unified Sensor

Code Configuration (air_quality_monitor_blynk_final.ino)

Before uploading the code, you must update the following four variables in the USER CONFIGURATION REQUIRED section:

auth[]: Your unique Blynk Auth Token (obtained when creating the project).

ssid[]: Your Wi-Fi network name.

pass[]: Your Wi-Fi network password.

R0_CLEAN_AIR: The unique, calibrated sensor resistance value determined during the calibration process (e.g., $258578.80$).

Key Code Features

PPM Calculation: Uses the $R_s/R_0$ ratio and a power-law curve to convert the MQ-135's resistance into a $\text{PPM}$ equivalent value.

Environmental Compensation: Includes a function (getCorrectionFactor) to adjust the $\text{PPM}$ reading based on the current Temperature and Humidity measured by the $\text{DHT}11$.

Passive Buzzer Control: Uses tone() and noTone() functions on pin $\text{D}0$ to generate a reliable alarm sound, which is necessary for 2-pin passive buzzers.

Tiered Alarm Logic

The sendSensorData function implements the dual-threshold logic:

// Tier 1: Low threshold for physical alarm (Buzzer & D6 LED)
const float PHYSICAL_ALARM_THRESHOLD_PPM = 50.0; 
// Tier 2: Higher threshold for the Blynk Virtual LED (V4)
const float BLYNK_VIRTUAL_LED_THRESHOLD_PPM = 250.0; 

// ... inside sendSensorData() ...

// TIER 1: PHYSICAL ALARM (Buzzer & Physical LED) - Triggers at 50 PPM
bool isPhysicalAlarm = (ppm > PHYSICAL_ALARM_THRESHOLD_PPM);
if (isPhysicalAlarm) {
  buzz(true); // Activate Buzzer
  digitalWrite(ALARM_LED_PIN, HIGH); // Activate Physical LED
} 

// TIER 2: BLYNK VIRTUAL LED (V4) - Triggers at 250 PPM
bool isBlynkAlert = (ppm > BLYNK_VIRTUAL_LED_THRESHOLD_PPM);
if (isBlynkAlert) {
  Blynk.virtualWrite(V4, 1); // Activate Blynk LED
} else {
  Blynk.virtualWrite(V4, 0); // Deactivate Blynk LED
}
