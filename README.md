## AS726X Sensor Library for ESP-IDF
This library provides a simple and efficient way to interface with the AS726X family of spectral sensors using the ESP-IDF framework. It allows you to easily integrate the AS726X sensor into your ESP32 projects for color sensing and spectral analysis applications.

### Features
*Easy Initialization*: Quickly initialize the sensor with customizable gain and measurement mode settings.
*Spectral Data Acquisition*: Read raw and calibrated spectral data from the sensor's channels.
*Temperature Reading*: Access the internal temperature sensor.
*LED Control*: Control the indicator LED and the bulb (illumination source) with adjustable current settings.
*Integration Time and Gain Adjustment*: Configure the sensor's integration time and gain for optimal measurements.
*Interrupt Handling*: Enable or disable interrupts for data-ready notifications.
*Soft Reset*: Reset the sensor to its default state programmatically.
### Supported Sensors
*AS7261*: Color sensor with XYZ, NIR, and clear channels.
*AS7262*: 6-channel visible light sensor (Violet, Blue, Green, Yellow, Orange, Red).
*AS7263*: 6-channel near-infrared sensor (R, S, T, U, V, W).
### Getting Started
_Prerequisites_
*ESP-IDF*: Ensure you have the ESP-IDF framework installed and set up for your ESP32 development environment.
*Hardware Connections*: Connect the AS726X sensor to your ESP32 via I2C. Default I2C pins are GPIO 21 (SDA) and GPIO 22 (SCL).
### Installation
-Clone the Repository: Clone or download this library into your project directory.
-Include the Library: Include the AS726X.h header file in your main application code.
### Usage Example
Below is a basic example demonstrating how to initialize the sensor and read spectral data.
```
#include "AS726X.h"

// I2C configuration
#define I2C_MASTER_SCL_IO          22    // SCL pin
#define I2C_MASTER_SDA_IO          21    // SDA pin
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000

void app_main(void)
{
    // Initialize I2C
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_num, &conf);
    i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);

    // Initialize AS726X sensor
    AS726X_t sensor;
    if (AS726X_init(&sensor, i2c_num, AS726X_ADDR, 3, 3)) {
        printf("Sensor initialized successfully.\n");
    } else {
        printf("Failed to initialize sensor.\n");
        return;
    }

    // Read spectral data
    if (AS726X_takeMeasurementsWithBulb(&sensor) == 0) {
        int violet = AS726X_getViolet(&sensor);
        int blue = AS726X_getBlue(&sensor);
        int green = AS726X_getGreen(&sensor);
        int yellow = AS726X_getYellow(&sensor);
        int orange = AS726X_getOrange(&sensor);
        int red = AS726X_getRed(&sensor);

        printf("Spectral Data:\n");
        printf("Violet: %d\n", violet);
        printf("Blue: %d\n", blue);
        printf("Green: %d\n", green);
        printf("Yellow: %d\n", yellow);
        printf("Orange: %d\n", orange);
        printf("Red: %d\n", red);
    } else {
        printf("Failed to take measurements.\n");
    }
}
```

### Important Functions
##### Initialization
*bool AS726X_init(AS726X_t *device, i2c_port_t port, uint8_t address, uint8_t gain, uint8_t measurement_mode)*
Initializes the sensor with the specified I2C port, address, gain, and measurement mode.

##### Spectral Data Acquisition

int AS726X_takeMeasurements(AS726X_t *device)
*Triggers a measurement and waits for data to be ready.*

int AS726X_takeMeasurementsWithBulb(AS726X_t *device)
*Enables the bulb, takes measurements, and then disables the bulb.*

int AS726X_getViolet(AS726X_t *device)
*Retrieves the raw violet channel data.*

int AS726X_getBlue(AS726X_t *device)
*Retrieves the raw blue channel data.*

_Similarly for other color channels:_

AS726X_getGreen(), AS726X_getYellow(), AS726X_getOrange(), AS726X_getRed()


##### Calibrated Data
float AS726X_getCalibratedViolet(AS726X_t *device)
*Retrieves the calibrated violet channel data.*

_Similarly for other channels_:

AS726X_getCalibratedBlue(), AS726X_getCalibratedGreen(), etc.

##### Temperature Reading
uint8_t AS726X_getTemperature(AS726X_t *device)
*Returns the internal sensor temperature in degrees Celsius.*

float AS726X_getTemperatureF(AS726X_t *device)
*Returns the internal sensor temperature in degrees Fahrenheit.*

##### LED Control

int AS726X_enableIndicator(AS726X_t *device)
*Turns on the indicator LED.*

int AS726X_disableIndicator(AS726X_t *device)
*Turns off the indicator LED.*

int AS726X_setIndicatorCurrent(AS726X_t *device, uint8_t current)*
*Sets the current for the indicator LED (0b00 to 0b11).*

int AS726X_enableBulb(AS726X_t *device)
*Turns on the bulb (illumination source).*

int AS726X_disableBulb(AS726X_t *device)
*Turns off the bulb.*

*int AS726X_setBulbCurrent(AS726X_t *device, uint8_t current)*
Sets the current for the bulb (0b00 to 0b11).

##### Configuration Settings
int AS726X_setIntegrationTime(AS726X_t *device, uint8_t integration_value)
*Sets the integration time (1 to 255), where time = value * 2.8ms.*

int AS726X_setGain(AS726X_t *device, uint8_t gain)
*Sets the gain setting (0 to 3).*

*int AS726X_setMeasurementMode(AS726X_t *device, uint8_t mode)*
Sets the measurement mode (0 to 3).

##### Interrupt Handling
int AS726X_enableInterrupt(AS726X_t *device)
*Enables interrupts for data-ready notifications.*

int AS726X_disableInterrupt(AS726X_t *device)
*Disables interrupts.*

##### Soft Reset
int AS726X_softReset(AS726X_t *device)
*Resets the sensor to its default state.*

Notes
I2C Address: The default I2C address for the AS726X sensor is 0x49. Ensure this matches your hardware configuration.
Pull-up Resistors: The I2C bus requires pull-up resistors. If your board doesn't have them, you'll need to add external resistors.
Measurement Timing: The time it takes to complete a measurement depends on the integration time and measurement mode settings.
License
This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.

Contributing
Contributions are welcome! Please submit a pull request or open an issue for any bugs or feature requests.

For questions or support, please open an issue on the repository.