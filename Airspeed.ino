/*
 * Airspeed and Temperature Display Program
 * 
 * This program reads differential pressure from an MS4525DO sensor,
 * calculates airspeed using Bernoulli's equation, and displays
 * the airspeed in MPH along with temperature in Fahrenheit
 * on a Grove 1.2-inch IPS Display.
 * 
 * Hardware:
 * - Arduino UNO / Seeeduino v4.2
 * - Grove Base Shield
 * - Grove 1.2-inch IPS Display
 * - MS4525DO Digital Air Speed Sensor
 * - Student built Pitot and static tubes
 */

#include "ms4525do.h"  // Bolder Flight MS4525DO library
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Arduino_ST7789_Fast.h> // Hardware-specific library for Grove 1.2-inch IPS Display
#include <Fonts/FreeSansBold18pt7b.h>  // Large bold font for values
#include <Fonts/FreeSans9pt7b.h>       // Smaller font for labels

// Define pins for the Grove 1.2-inch IPS Display
#define SCK   7  // Clock pin for Grove display
#define SDA   8  // Data pin for Grove display

// Create instances for the sensor and display
bfs::Ms4525do pres;  // MS4525DO pressure sensor
Arduino_ST7789 lcd = Arduino_ST7789(SCK, SDA);  // Grove 1.2-inch IPS Display

// Define colors for the display
#define BLACK 0x0000
#define BLUE  0x001F
#define RED   0xF800
#define WHITE 0xFFFF
#define GREEN 0x0320

// Variables for airspeed and temperature
float airSpeedMS = 0.0;
float airSpeedMPH = 0.0;
float Temp_F = 0.0;

// Calibration and filtering variables
float zero_pressure_offset = 0.0;
bool calibrated = false;
const int calibration_samples = 50;
const float alpha = 0.2; // Filter coefficient (0-1): lower = smoother but slower response
float filteredPressure = 0.0;
float filteredAirspeed = 0.0;

// Function for calibrating the pressure sensor at startup
void calibratePressureSensor() {
  Serial.println("Calibrating pressure sensor...");
  Serial.println("Ensure zero airflow during calibration!");
  
  // Display calibration message
  lcd.fillScreen(BLACK);
  lcd.setTextColor(WHITE, BLACK);
  lcd.setFont(&FreeSans9pt7b);
  lcd.setCursor(20, 120);
  lcd.println("Calibrating...");
  
  float sum = 0.0;
  int valid_samples = 0;
  
  // Take multiple samples to average out noise
  for (int i = 0; i < calibration_samples; i++) {
    if (pres.Read()) {
      sum += -1.0 * pres.pres_pa();  // Use same inversion as in main loop
      valid_samples++;
    }
    delay(20);  // Short delay between samples
    
    // Show progress
    if (i % 10 == 0) {
      lcd.fillRect(20, 140, 200, 20, BLACK);
      lcd.setCursor(20, 160);
      lcd.print("Progress: ");
      lcd.print(i * 100 / calibration_samples);
      lcd.print("%");
    }
  }
  
  if (valid_samples > 0) {
    zero_pressure_offset = sum / valid_samples;
    calibrated = true;
    Serial.print("Calibration complete. Zero pressure offset: ");
    Serial.println(zero_pressure_offset, 6);
    
    // Show calibration result
    lcd.fillRect(20, 140, 200, 20, BLACK);
    lcd.setCursor(20, 160);
    lcd.print("Offset: ");
    lcd.print(zero_pressure_offset, 2);
    lcd.print(" Pa");
  } else {
    Serial.println("Calibration failed! Check sensor connection.");
    lcd.fillRect(20, 140, 200, 20, BLACK);
    lcd.setCursor(20, 160);
    lcd.setTextColor(RED, BLACK);
    lcd.print("Calibration Failed!");
  }
  
  delay(1000);  // Show calibration result for a second
}

// Function to calculate air density based on temperature
float calculateAirDensity(float temperature_c, float pressure_pa) {
  // Constants
  const float R = 287.05; // Specific gas constant for dry air (J/(kg·K))
  const float P0 = 101325.0; // Standard pressure at sea level (Pa)
  
  // Convert temperature to Kelvin
  float temperature_k = temperature_c + 273.15;
  
  // Use ambient atmospheric pressure
  float ambient_pressure_pa = P0; // For more accuracy, use actual atmospheric pressure
  
  // Calculate density using the ideal gas law: ρ = P/(R·T)
  return ambient_pressure_pa / (R * temperature_k);
}

// Function for temperature compensation of pressure readings
float compensatePressure(float rawPressure, float temperature_c) {
  // This is a simplified example - you'd need to characterize your specific sensor
  float nominal_temp = 25.0; // Nominal calibration temperature in C
  float temp_coefficient = 0.0002; // Example coefficient - determined experimentally
  
  return rawPressure * (1 + temp_coefficient * (temperature_c - nominal_temp));
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize the sensor
  // Initialize the MS4525DO sensor (I2C address 0x28, -1 to 1 PSI range)
  pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  
  // Initialize the MS4525DO sensor
  if (!pres.Begin()) {
    Serial.println("Error connecting to MS4525DO sensor");
    while(1) {}
  }
  
  // Initialize the display
  lcd.init();
  Serial.println("Display initialized");
  
  // Set initial display settings
  lcd.fillScreen(BLACK);
  lcd.setTextSize(1);  // Set to 1 when using custom fonts
  
  // Display startup message to verify display is working
  lcd.setTextColor(RED, BLACK);
  lcd.setFont(&FreeSans9pt7b);
  lcd.setCursor(20, 120);
  lcd.println("Ready");
  delay(1000);
  
  // Perform sensor calibration after a short delay
  delay(1000);  // Allow sensor to stabilize
  calibratePressureSensor();
  
  // Initialize display with static elements
  initializeDisplay();
}

void loop() {
  // Read data from the MS4525DO sensor
  if (pres.Read()) {
    // Get pressure reading and apply offset correction
    float raw_pressure_pa = -1.0 * pres.pres_pa();  // Keep inversion
    float pressure_pa = raw_pressure_pa - zero_pressure_offset;  // Apply calibration offset
    
    // Apply temperature compensation
    float compensated_pressure = compensatePressure(pressure_pa, pres.die_temp_c());
    
    // Apply low-pass filter to pressure reading
    filteredPressure = (alpha * compensated_pressure) + ((1 - alpha) * filteredPressure);
    
    // Debug output
    Serial.print("Raw Pressure (Pa): ");
    Serial.print(raw_pressure_pa, 6);
    Serial.print("\tCorrected Pressure (Pa): ");
    Serial.print(compensated_pressure, 6);
    Serial.print("\tFiltered Pressure (Pa): ");
    Serial.println(filteredPressure, 6);
    
    // Calculate air density based on temperature
    float airDensity = calculateAirDensity(pres.die_temp_c(), 101325.0);
    
    // Handle both positive and negative pressure readings
    if (abs(filteredPressure) < 0.5) {  // Small threshold to eliminate noise
      // Zero pressure means zero airspeed
      airSpeedMS = 0;
      airSpeedMPH = 0;
    } else {
      // Determine the sign of the pressure (direction of airflow)
      float pressureSign = (filteredPressure > 0) ? 1.0 : -1.0;
      
      // Calculate airspeed magnitude using absolute pressure value
      float absPressure = abs(filteredPressure);
      
      // Calculate airspeed in m/s using Bernoulli's equation and calculated air density
      airSpeedMS = pressureSign * sqrt((2 * absPressure) / airDensity);
      
      // Convert from m/s to mph
      airSpeedMPH = airSpeedMS * 2.236;
    }
    
    // Apply low-pass filter to airspeed
    filteredAirspeed = (alpha * airSpeedMPH) + ((1 - alpha) * filteredAirspeed);
    
    // Convert temperature from C to F
    Temp_F = (pres.die_temp_c() * 9.0 / 5.0) + 32.0;
    
    // Serial output for diagnostic purposes
    Serial.print("Airspeed (m/s): ");
    Serial.print(airSpeedMS, 3);
    Serial.print("\t");
    Serial.print("Airspeed (mph): ");
    Serial.print(airSpeedMPH, 3);
    Serial.print("\t");
    Serial.print("Filtered Airspeed (mph): ");
    Serial.print(filteredAirspeed, 3);
    Serial.print("\t");
    Serial.print("Temperature (C): ");
    Serial.print(pres.die_temp_c(), 3);
    Serial.print("\t");
    Serial.print("Temperature (F): ");
    Serial.println(Temp_F, 3);
    
    // Check for invalid or unlikely values
    if (abs(filteredPressure) > 5000) { // Pressure value unreasonably high
      Serial.println("WARNING: Pressure reading out of expected range");
      lcd.fillRect(0, 0, 50, 20, BLACK);
      lcd.setCursor(5, 15);
      lcd.setTextColor(RED);
      lcd.print("ERROR");
    }
    
    // Update the display
    updateDisplay();
  } else {
    Serial.println("Error reading from sensor");
  }
  
  // Small delay to prevent excessive updates
  delay(100);
}

// Variables to track previous values for partial updates
float prevAirspeedValue = -999.9;  // Set to an extreme value to force update on first pass
float prevAirspeedFillAngle = 0;
float prevTempValue = -999.9;      // Set to an extreme value to force update on first pass
float prevTempFillAngle = 0;

// Function to initialize a gauge (draw the static elements)
void initGauge(int centerX, int centerY, int radius, uint16_t emptyColor, const char* label) {
  int arcThickness = 15;  // Thickness of the arc in pixels
  int innerRadius = radius - arcThickness;
  
  // Draw the empty half-circle background arc (180 degrees, from 0 to 180)
  for (int r = innerRadius; r <= radius; r++) {
    for (int i = 0; i <= 180; i += 1) {
      float angle = i * PI / 180.0;
      int x = centerX - r * cos(angle);  // Negative cos for 0-180 range
      int y = centerY - r * sin(angle);
      lcd.drawPixel(x, y, emptyColor);
    }
  }
  
  // Display the label
  lcd.setFont(&FreeSans9pt7b);
  lcd.setTextColor(emptyColor, BLACK);
  
  // Count actual characters in the label
  int labelCharCount = strlen(label);
  
  // Calculate width based on character count
  // Each character is approximately 9 pixels wide in FreeSans9pt7b
  int labelCharWidth = 9;
  int labelTotalWidth = labelCharCount * labelCharWidth;
  
  // Center based on calculated width
  lcd.setCursor(centerX - (labelTotalWidth / 2), centerY + -15); // Position below the gauge
  lcd.print(label);
}

// Function to update a gauge's value (only update the dynamic elements)
void updateGaugeValue(int centerX, int centerY, int radius, float minValue, float maxValue, float value, float &prevValue, float &prevFillAngle, uint16_t emptyColor, uint16_t fillColor) {
  // If the value hasn't changed significantly, don't update
  if (abs(value - prevValue) < 0.2) return;
  
  // Calculate the angle for the filled portion of the arc (0-180 degrees)
  float valueRange = maxValue - minValue;
  float valuePercent = (value - minValue) / valueRange;
  valuePercent = constrain(valuePercent, 0.0, 1.0); // Ensure value is within range
  float fillAngle = valuePercent * 180;
  
  // Define arc thickness
  int arcThickness = 15;
  
  // Only update the arc if it has changed significantly
  if (abs(fillAngle - prevFillAngle) > 1.0) {
    // If previous value was higher, redraw the empty arc where needed
    if (prevFillAngle > fillAngle) {
      for (int r = radius - arcThickness; r <= radius; r++) {
        for (int i = fillAngle + 1; i <= prevFillAngle; i += 1) {
          float angle = i * PI / 180.0;
          int x = centerX - r * cos(angle);
          int y = centerY - r * sin(angle);
          lcd.drawPixel(x, y, emptyColor);
        }
      }
    }
    
    // If new value is higher, draw the filled arc where needed
    if (fillAngle > prevFillAngle) {
      for (int r = radius - arcThickness; r <= radius; r++) {
        for (int i = prevFillAngle + 1; i <= fillAngle; i += 1) {
          float angle = i * PI / 180.0;
          int x = centerX - r * cos(angle);
          int y = centerY - r * sin(angle);
          lcd.drawPixel(x, y, fillColor);
        }
      }
    }
    
    // Update the previous fill angle
    prevFillAngle = fillAngle;
  }
  
  // Clear only the digit area with a black rectangle
  int digitAreaWidth = 100;
  int digitAreaHeight = 30;
  lcd.fillRect(centerX - (digitAreaWidth / 2), centerY - 5, digitAreaWidth, digitAreaHeight, BLACK);
  
  // Format value with consistent width based on its magnitude
  char valueStr[10];
  if (value >= 100.0) {
    // For values 100 and above, format with 1 decimal place
    dtostrf(value, 5, 1, valueStr);
  } else {
    // For values below 100, format with fixed width for XX.X format
    dtostrf(value, 4, 1, valueStr);
  }
  
  // Trim leading spaces
  char* trimmedStr = valueStr;
  while (*trimmedStr == ' ') trimmedStr++;
  
  // Display the new value
  lcd.setFont(&FreeSans9pt7b);  // Use the non-bold font for a thinner appearance
  lcd.setTextColor(fillColor, BLACK);
  
  // Count actual characters (including decimal point)
  int charCount = strlen(trimmedStr);
  
  // Calculate width based on character count
  int digitWidth = 9;  // Each digit is approximately 9 pixels wide in FreeSans9pt7b
  int totalWidth = charCount * digitWidth;
  
  // Center based on calculated width
  lcd.setCursor(centerX - (totalWidth / 2), centerY + 10);
  lcd.print(trimmedStr);
  
  // Update the previous value
  prevValue = value;
  
  // Add a direction indicator if airspeed is significant (for airspeed only)
  if (centerY == 70 && abs(value) > 5.0) { // This is the airspeed gauge and speed > 5 MPH
    uint16_t dirColor = (airSpeedMS > 0) ? GREEN : RED; // Green for positive, red for negative
    lcd.fillTriangle(
      120, 45, // Tip of the triangle
      110, 35, // Bottom left
      130, 35, // Bottom right
      dirColor
    );
  }
}

// Flag to track if display has been initialized
bool displayInitialized = false;

void initializeDisplay() {
  // Clear the display
  lcd.fillScreen(BLACK);
  
  // Initialize the static parts of the gauges
  initGauge(120, 70, 70, WHITE, "MPH  ");  // Airspeed gauge with white background - moved up and made slightly smaller
  initGauge(120, 210, 70, WHITE, "Temp F");  // Temperature gauge with white background - moved down and made slightly smaller
  
  // Clear the areas where digits will be displayed to ensure no remnants
  lcd.fillRect(120 - 50, 70 - 5, 100, 30, BLACK);  // Clear airspeed digit area
  lcd.fillRect(120 - 50, 210 - 5, 100, 30, BLACK); // Clear temperature digit area
  
  // Mark display as initialized
  displayInitialized = true;
}

void updateDisplay() {
  // Print debug info to serial
  Serial.println("Updating display");
  
  // Initialize the display if it's the first time
  if (!displayInitialized) {
    initializeDisplay();
  }
  
  // Use filtered airspeed for the display
  float maxSpeed = 150.0; // Maximum speed on the gauge
  float displaySpeed = constrain(filteredAirspeed, 0.0, maxSpeed); // Use filtered airspeed
  updateGaugeValue(120, 70, 70, 0, maxSpeed, displaySpeed, prevAirspeedValue, prevAirspeedFillAngle, WHITE, GREEN);
  
  // Update only the dynamic parts of the temperature gauge
  float minTemp = 0.0;
  float maxTemp = 100.0;
  float displayTemp = constrain(Temp_F, minTemp, maxTemp);
  updateGaugeValue(120, 210, 70, minTemp, maxTemp, displayTemp, prevTempValue, prevTempFillAngle, WHITE, RED);
}