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

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while(!Serial) {}
  
  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(400000);
  
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
  lcd.setFont(&FreeSansBold18pt7b);
  lcd.setCursor(20, 120);
  lcd.println("Ready");
  delay(1000);
  
  // Initialize display with static elements
  initializeDisplay();
}

void loop() {
  // Read data from the MS4525DO sensor
  if (pres.Read()) {
    // Get pressure reading
    float pressure_pa = -1.0 * pres.pres_pa();  // Invert sign to correct for reversed connections
    
    // Debug output
    Serial.print("Raw Pressure (Pa): ");
    Serial.println(pressure_pa, 6);
    
    // Handle both positive and negative pressure readings
    if (pressure_pa == 0) {
      // Zero pressure means zero airspeed
      airSpeedMS = 0;
      airSpeedMPH = 0;
    } else {
      // Determine the sign of the pressure (direction of airflow)
      float pressureSign = (pressure_pa > 0) ? 1.0 : -1.0;
      
      // Calculate airspeed magnitude using absolute pressure value
      float absPressure = abs(pressure_pa);
      
      // Calculate airspeed in m/s using Bernoulli's equation
      airSpeedMS = pressureSign * sqrt((2 * absPressure) / 1.2041);  // 1.2041 is air density at sea level
      
      // Convert from m/s to mph
      airSpeedMPH = airSpeedMS * 2.236;
    }
    
    // Convert temperature from C to F
    Temp_F = (pres.die_temp_c() * 9.0 / 5.0) + 32.0;
    
    // Serial output for diagnostic purposes
    Serial.print("Pressure (Pa): ");
    Serial.print(pressure_pa, 6);
    Serial.print("\t");
    Serial.print("Airspeed (m/s): ");
    Serial.print(airSpeedMS, 3);
    Serial.print("\t");
    Serial.print("Airspeed (mph): ");
    Serial.print(airSpeedMPH, 3);
    Serial.print("\t");
    Serial.print("Temperature (C): ");
    Serial.print(pres.die_temp_c(), 3);
    Serial.print("\t");
    Serial.print("Temperature (F): ");
    Serial.println(Temp_F, 3);
    
    // Update the display
    updateDisplay();
  }
  
  // Small delay to prevent excessive updates
  delay(100);
}

// Variables to track previous values for partial updates
float prevAirspeedValue = -1;
float prevTempValue = -1;

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
  
  // Draw tick marks on the gauge
  for (int i = 0; i <= 180; i += 30) {
    float angle = i * PI / 180.0;
    int x1 = centerX - (radius + 5) * cos(angle);
    int y1 = centerY - (radius + 5) * sin(angle);
    int x2 = centerX - (radius - arcThickness - 5) * cos(angle);
    int y2 = centerY - (radius - arcThickness - 5) * sin(angle);
    lcd.drawLine(x1, y1, x2, y2, emptyColor);
  }
  
  // Display the label
  lcd.setFont(&FreeSans9pt7b);
  lcd.setTextColor(emptyColor, BLACK);
  int textWidth = strlen(label) * 6; // Approximate width
  lcd.setCursor(centerX - textWidth/2, centerY + 10);
  lcd.print(label);
}

// Function to update a gauge's value (only update the dynamic elements)
void updateGaugeValue(int centerX, int centerY, int radius, float minValue, float maxValue, float value, 
                      uint16_t emptyColor, uint16_t fillColor, float &prevValue) {
  // If value hasn't changed significantly, don't update
  if (abs(value - prevValue) < 0.2) return;
  
  int arcThickness = 15;  // Thickness of the arc in pixels
  int innerRadius = radius - arcThickness;
  
  // Calculate the angle for the current and previous values
  float valueRange = maxValue - minValue;
  float valuePercent = (value - minValue) / valueRange;
  valuePercent = constrain(valuePercent, 0.0, 1.0); // Ensure value is within range
  int fillAngle = valuePercent * 180;
  
  float prevPercent = (prevValue - minValue) / valueRange;
  prevPercent = constrain(prevPercent, 0.0, 1.0);
  int prevFillAngle = prevPercent * 180;
  
  // Clear the value display area with a black rectangle
  int textHeight = 30; // Approximate height for the large font
  int textWidth = 80;  // Approximate width for the value text
  lcd.fillRect(centerX - textWidth/2, centerY - 20 - textHeight + 5, textWidth, textHeight, BLACK);
  
  // If previous value was higher, redraw the empty arc where needed
  if (prevFillAngle > fillAngle) {
    for (int r = innerRadius; r <= radius; r++) {
      for (int i = fillAngle + 1; i <= prevFillAngle; i += 1) {
        float angle = i * PI / 180.0;
        int x = centerX - r * cos(angle);
        int y = centerY - r * sin(angle);
        lcd.drawPixel(x, y, emptyColor);
      }
    }
  }
  
  // Draw the filled portion of the arc where needed
  if (fillAngle > prevFillAngle) {
    for (int r = innerRadius; r <= radius; r++) {
      for (int i = prevFillAngle + 1; i <= fillAngle; i += 1) {
        float angle = i * PI / 180.0;
        int x = centerX - r * cos(angle);
        int y = centerY - r * sin(angle);
        lcd.drawPixel(x, y, fillColor);
      }
    }
  }
  
  // Display the new value
  lcd.setFont(&FreeSansBold18pt7b);
  lcd.setTextColor(fillColor, BLACK);
  char valueStr[10];
  dtostrf(value, 4, 1, valueStr);
  textWidth = strlen(valueStr) * 12; // Approximate width for larger font
  lcd.setCursor(centerX - textWidth/2, centerY - 20);
  lcd.print(valueStr);
  
  // Update the previous value
  prevValue = value;
}

// Flag to track if display has been initialized
bool displayInitialized = false;

void initializeDisplay() {
  // Clear the display
  lcd.fillScreen(BLACK);
  
  // Initialize the static parts of the gauges
  initGauge(120, 90, 80, WHITE, "MPH");  // Airspeed gauge with white background
  initGauge(120, 190, 80, WHITE, "Temp F");  // Temperature gauge with white background
  
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
  
  // Update only the dynamic parts of the airspeed gauge
  float maxSpeed = 200.0; // Maximum speed on the gauge
  float displaySpeed = constrain(airSpeedMPH, 0.0, maxSpeed); // Constrain to positive values for gauge
  updateGaugeValue(120, 90, 80, 0, maxSpeed, displaySpeed, WHITE, GREEN, prevAirspeedValue);
  
  // Update only the dynamic parts of the temperature gauge
  float minTemp = 0.0;
  float maxTemp = 100.0;
  float displayTemp = constrain(Temp_F, minTemp, maxTemp);
  updateGaugeValue(120, 190, 80, minTemp, maxTemp, displayTemp, WHITE, RED, prevTempValue);
}
