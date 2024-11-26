/*
 * AS726X.h
 * LManuXX Library for AS726X spectral sensor
 */

#ifndef _AS726X_H_
#define _AS726X_H_

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AS726X_ADDR 0x49 // I2C address del sensor

// register addresses
#define AS726x_DEVICE_TYPE      0x00
#define AS726x_HW_VERSION       0x01
#define AS726x_CONTROL_SETUP    0x04
#define AS726x_INT_T            0x05
#define AS726x_DEVICE_TEMP      0x06
#define AS726x_LED_CONTROL      0x07

#define AS72XX_SLAVE_STATUS_REG 0x00
#define AS72XX_SLAVE_WRITE_REG  0x01
#define AS72XX_SLAVE_READ_REG   0x02

// registers for AS7262
#define AS7262_V        0x08
#define AS7262_B        0x0A
#define AS7262_G        0x0C
#define AS7262_Y        0x0E
#define AS7262_O        0x10
#define AS7262_R        0x12
#define AS7262_V_CAL    0x14
#define AS7262_B_CAL    0x18
#define AS7262_G_CAL    0x1C
#define AS7262_Y_CAL    0x20
#define AS7262_O_CAL    0x24
#define AS7262_R_CAL    0x28

// registers for AS7263
#define AS7263_R        0x08
#define AS7263_S        0x0A
#define AS7263_T        0x0C
#define AS7263_U        0x0E
#define AS7263_V        0x10
#define AS7263_W        0x12
#define AS7263_R_CAL    0x14
#define AS7263_S_CAL    0x18
#define AS7263_T_CAL    0x1C
#define AS7263_U_CAL    0x20
#define AS7263_V_CAL    0x24
#define AS7263_W_CAL    0x28

// registers for AS7261
#define AS7261_X           0x08 // 16 bits
#define AS7261_Y           0x0A // 16 bits
#define AS7261_Z           0x0C // 16 bits
#define AS7261_NIR         0x0E // 16 bits
#define AS7261_DARK        0x10 // 16 bits
#define AS7261_CLEAR       0x12 // 16 bits
#define AS7261_X_CAL       0x14
#define AS7261_Y_CAL       0x18
#define AS7261_Z_CAL       0x1C
#define AS7261_X1931_CAL   0x20
#define AS7261_Y1931_CAL   0x24
#define AS7261_UPRI_CAL    0x28
#define AS7261_VPRI_CAL    0x2C
#define AS7261_U_CAL       0x30
#define AS7261_V_CAL       0x34
#define AS7261_DUV_CAL     0x38
#define AS7261_LUX_CAL     0x3C // 16 bits
#define AS7261_CCT_CAL     0x3E // 16 bits

#define AS72XX_SLAVE_TX_VALID 0x02
#define AS72XX_SLAVE_RX_VALID 0x01

#define SENSORTYPE_AS7261 0x3D
#define SENSORTYPE_AS7262 0x3E
#define SENSORTYPE_AS7263 0x3F

#define POLLING_DELAY 5 // milliseconds of delay between polling the sensor for data
#define MAX_RETRIES 3
#define TIMEOUT 3000

// strctu for AS726X sensor
typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    uint8_t gain;
    uint8_t measurement_mode;
    uint8_t sensor_version;
} AS726X_t;

bool AS726X_init(AS726X_t *device, i2c_port_t port, uint8_t address, uint8_t gain, uint8_t measurement_mode);
int AS726X_takeMeasurements(AS726X_t *device);
uint8_t AS726X_getVersion(AS726X_t *device);
int AS726X_takeMeasurementsWithBulb(AS726X_t *device);
uint8_t AS726X_getTemperature(AS726X_t *device);
float AS726X_getTemperatureF(AS726X_t *device);
int AS726X_setMeasurementMode(AS726X_t *device, uint8_t mode);
uint8_t AS726X_getMeasurementMode(AS726X_t *device);
bool AS726X_dataAvailable(AS726X_t *device);
int AS726X_enableIndicator(AS726X_t *device);
int AS726X_disableIndicator(AS726X_t *device);
int AS726X_setIndicatorCurrent(AS726X_t *device, uint8_t current);
int AS726X_enableBulb(AS726X_t *device);
int AS726X_disableBulb(AS726X_t *device);
int AS726X_setBulbCurrent(AS726X_t *device, uint8_t current);
int AS726X_softReset(AS726X_t *device);
int AS726X_setGain(AS726X_t *device, uint8_t gain);
uint8_t AS726X_getGain(AS726X_t *device);
int AS726X_setIntegrationTime(AS726X_t *device, uint8_t integration_value);
uint8_t AS726X_getIntegrationTime(AS726X_t *device);
int AS726X_enableInterrupt(AS726X_t *device);
int AS726X_disableInterrupt(AS726X_t *device);

int AS726X_getViolet(AS726X_t *device);
int AS726X_getBlue(AS726X_t *device);
int AS726X_getGreen(AS726X_t *device);
int AS726X_getYellow(AS726X_t *device);
int AS726X_getOrange(AS726X_t *device);
int AS726X_getRed(AS726X_t *device);

int AS726X_getR(AS726X_t *device);
int AS726X_getS(AS726X_t *device);
int AS726X_getT(AS726X_t *device);
int AS726X_getU(AS726X_t *device);
int AS726X_getV(AS726X_t *device);
int AS726X_getW(AS726X_t *device);

int AS726X_getX(AS726X_t *device);
int AS726X_getY(AS726X_t *device);
int AS726X_getZ(AS726X_t *device);
int AS726X_getNIR(AS726X_t *device);
int AS726X_getDark(AS726X_t *device);
int AS726X_getClear(AS726X_t *device);

float AS726X_getCalibratedViolet(AS726X_t *device);
float AS726X_getCalibratedBlue(AS726X_t *device);
float AS726X_getCalibratedGreen(AS726X_t *device);
float AS726X_getCalibratedYellow(AS726X_t *device);
float AS726X_getCalibratedOrange(AS726X_t *device);
float AS726X_getCalibratedRed(AS726X_t *device);

float AS726X_getCalibratedR(AS726X_t *device);
float AS726X_getCalibratedS(AS726X_t *device);
float AS726X_getCalibratedT(AS726X_t *device);
float AS726X_getCalibratedU(AS726X_t *device);
float AS726X_getCalibratedV(AS726X_t *device);
float AS726X_getCalibratedW(AS726X_t *device);

float AS726X_getCalibratedX(AS726X_t *device);
float AS726X_getCalibratedY(AS726X_t *device);
float AS726X_getCalibratedZ(AS726X_t *device);
float AS726X_getCalibratedX1931(AS726X_t *device);
float AS726X_getCalibratedY1931(AS726X_t *device);
float AS726X_getCalibratedUPri1976(AS726X_t *device);
float AS726X_getCalibratedVPri1976(AS726X_t *device);
float AS726X_getCalibratedU1976(AS726X_t *device);
float AS726X_getCalibratedV1976(AS726X_t *device);
float AS726X_getCalibratedDUV1976(AS726X_t *device);
int AS726X_getCalibratedLux(AS726X_t *device);
int AS726X_getCalibratedCCT(AS726X_t *device);

#ifdef __cplusplus
}
#endif

#endif // _AS726X_H_
