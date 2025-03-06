# Airspeed and Temperature Display

This Arduino project reads differential pressure from an MS4525DO sensor, calculates airspeed using Bernoulli's equation, and displays the airspeed in MPH along with temperature in Fahrenheit on a Grove 1.2-inch IPS Display. The display features attractive half-circle gauge visualizations for both measurements.

## Hardware Requirements

- Arduino UNO / Seeeduino v4.2
- Grove Base Shield
- Grove 1.2-inch IPS Display
- MS4525DO Digital Air Speed Sensor (0x28 I2C address, -1 to 1 PSI range)
- Student built Pitot and static tubes (optional)
- Jumper wires for connections

## Wiring Connections

### MS4525DO Sensor
- VCC → 5V
- GND → GND
- SCL → SCL (A5 on Arduino UNO/Seeeduino)
- SDA → SDA (A4 on Arduino UNO/Seeeduino)

### Grove 1.2-inch IPS Display
- Connect to the Grove Base Shield using the provided Grove cable
- The code uses pins 7 (SCK) and 8 (SDA) for the display connection

## Software Requirements

### Arduino IDE Setup
1. Download and install the [Arduino IDE](https://www.arduino.cc/en/software)
2. Add Seeeduino boards to the Arduino IDE:
   - Open Arduino IDE
   - Go to File → Preferences
   - Add the following URL to the "Additional Boards Manager URLs" field:
     ```
     https://raw.githubusercontent.com/Seeed-Studio/Seeed_Platform/master/package_seeeduino_boards_index.json
     ```
   - Go to Tools → Board → Boards Manager
   - Search for "Seeeduino" and install the package

3. Select the correct board:
   - Tools → Board → Seeeduino AVR → Seeeduino V4.2/4.3

### Required Libraries
Install the following libraries through the Arduino Library Manager (Tools → Manage Libraries):

1. **Wire** (built-in with Arduino IDE)
2. **Adafruit GFX Library** - Core graphics library
   - Search for "Adafruit GFX" and install
   
3. **Arduino_ST7789_Fast** - Hardware-specific library for Grove 1.2-inch IPS Display
   - Search for "Arduino_ST7789_Fast" and install
   
4. **Adafruit BusIO** - Required by Adafruit GFX
   - Search for "Adafruit BusIO" and install

### Custom MS4525DO Library
The project uses a custom MS4525DO library that's included in the repository. Make sure the following files are in your project folder:
- `ms4525do.h`
- `ms4525do.cpp`

## Project Files

- `airspeed_display.ino` - Main Arduino sketch
- `ms4525do.h` - Header file for the MS4525DO sensor
- `ms4525do.cpp` - Implementation file for the MS4525DO sensor
- `README.md` - This documentation file

## Features

- Real-time airspeed calculation using Bernoulli's equation
- Temperature display in Fahrenheit
- Half-circle gauge visualization for both airspeed and temperature
- Efficient display updates that only refresh changed portions of the screen
- Airspeed range: 0-200 MPH
- Temperature range: 0-100°F

## Usage

1. Connect all hardware as described in the Wiring Connections section
2. Open the `airspeed_display.ino` file in Arduino IDE
3. Verify that all required libraries are installed
4. Select the correct board and port from the Tools menu
5. Upload the sketch to your Arduino/Seeeduino
6. The display will show "Ready" briefly, then display the gauges
7. Blow into the pitot tube or apply pressure to the sensor to see the airspeed gauge change

## Customization

You can modify the following parameters in the code to customize the display:

- Gauge ranges: Change `maxSpeed` and `maxTemp`/`minTemp` variables
- Gauge colors: Modify the color definitions for the arcs
- Gauge positions: Adjust the centerX, centerY, and radius parameters
- Update threshold: Change the `0.2` value in the `updateGaugeValue` function to make the display more or less sensitive to small changes

## Troubleshooting

- If the display doesn't initialize, check the wiring connections and make sure the correct pins are defined in the code
- If the sensor doesn't respond, verify the I2C address (default is 0x28) and check the I2C connections
- For inverted airspeed readings, you may need to adjust the sign of the pressure reading in the code

## License

This project is open source and available under the MIT License.

## Credits

- Adafruit for the GFX Library
- Bolder Flight Systems for the MS4525DO sensor code base
