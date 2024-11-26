/*
 * AS726X.c
 * Implementación de las funciones para el sensor AS726X
 */

#include "AS726X.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "AS726X";

static uint8_t virtualReadRegister(AS726X_t *device, uint8_t virtualAddr);
static int virtualWriteRegister(AS726X_t *device, uint8_t virtualAddr, uint8_t dataToWrite);
static uint8_t readRegister(AS726X_t *device, uint8_t addr);
static int writeRegister(AS726X_t *device, uint8_t addr, uint8_t val);
static int clearDataAvailable(AS726X_t *device);
static int getChannel(AS726X_t *device, uint8_t channelRegister);
static float getCalibratedValue(AS726X_t *device, uint8_t calAddress);
static float convertBytesToFloat(uint32_t myLong);


bool AS726X_init(AS726X_t *device, i2c_port_t port, uint8_t address, uint8_t gain, uint8_t measurement_mode) {
    device->i2c_port = port;
    device->i2c_addr = address;
    device->gain = gain;
    device->measurement_mode = measurement_mode;

    device->sensor_version = virtualReadRegister(device, AS726x_HW_VERSION);

    if (device->sensor_version != SENSORTYPE_AS7261 &&
        device->sensor_version != SENSORTYPE_AS7262 &&
        device->sensor_version != SENSORTYPE_AS7263) {
        ESP_LOGE(TAG, "Versión del sensor no compatible: 0x%02X", device->sensor_version);
        return false;
    }

    if (AS726X_setBulbCurrent(device, 0b00) != 0) return false;
    if (AS726X_disableBulb(device) != 0) return false;
    if (AS726X_setIndicatorCurrent(device, 0b11) != 0) return false;
    if (AS726X_disableIndicator(device) != 0) return false;
    if (AS726X_setIntegrationTime(device, 50) != 0) return false;
    if (AS726X_setGain(device, gain) != 0) return false;
    if (AS726X_setMeasurementMode(device, measurement_mode) != 0) return false;

    return true;
}

uint8_t AS726X_getVersion(AS726X_t *device) {
    return device->sensor_version;
}

int AS726X_setMeasurementMode(AS726X_t *device, uint8_t mode) {
    if (mode > 0b11) mode = 0b11;

    uint8_t value = virtualReadRegister(device, AS726x_CONTROL_SETUP);
    value &= 0b11110011;
    value |= (mode << 2);
    return virtualWriteRegister(device, AS726x_CONTROL_SETUP, value);
}

uint8_t AS726X_getMeasurementMode(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_CONTROL_SETUP);
    return (value & 0b00001100) >> 2;
}

int AS726X_setGain(AS726X_t *device, uint8_t gain) {
    if (gain > 0b11) gain = 0b11;

    uint8_t value = virtualReadRegister(device, AS726x_CONTROL_SETUP);
    value &= 0b11001111;
    value |= (gain << 4);
    return virtualWriteRegister(device, AS726x_CONTROL_SETUP, value);
}

uint8_t AS726X_getGain(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_CONTROL_SETUP);
    return (value & 0b00110000) >> 4;
}

int AS726X_setIntegrationTime(AS726X_t *device, uint8_t integration_value) {
    return virtualWriteRegister(device, AS726x_INT_T, integration_value);
}

uint8_t AS726X_getIntegrationTime(AS726X_t *device) {
    return virtualReadRegister(device, AS726x_INT_T);
}

int AS726X_enableInterrupt(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_CONTROL_SETUP);
    value |= 0b01000000;
    return virtualWriteRegister(device, AS726x_CONTROL_SETUP, value);
}

int AS726X_disableInterrupt(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_CONTROL_SETUP);
    value &= 0b10111111;
    return virtualWriteRegister(device, AS726x_CONTROL_SETUP, value);
}

int AS726X_takeMeasurements(AS726X_t *device) {
    if (clearDataAvailable(device) != 0) return -1;

    if (AS726X_setMeasurementMode(device, 3) != 0) return -1;

    uint32_t start_time = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(TIMEOUT);

    while (!AS726X_dataAvailable(device)) {
        vTaskDelay(pdMS_TO_TICKS(POLLING_DELAY));
        if ((xTaskGetTickCount() - start_time) > timeout_ticks) {
            ESP_LOGE(TAG, "Timeout esperando a que los datos estén disponibles");
            return -1;
        }
    }

    return 0;
}

int AS726X_takeMeasurementsWithBulb(AS726X_t *device) {
    if (AS726X_enableBulb(device) != 0) return -1;

    if (AS726X_takeMeasurements(device) != 0) return -1;

    if (AS726X_disableBulb(device) != 0) return -1;

    return 0;
}

bool AS726X_dataAvailable(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_CONTROL_SETUP);
    return (value & (1 << 1)) != 0;
}

int AS726X_enableIndicator(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_LED_CONTROL);
    value |= (1 << 0);
    return virtualWriteRegister(device, AS726x_LED_CONTROL, value);
}

int AS726X_disableIndicator(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_LED_CONTROL);
    value &= ~(1 << 0);
    return virtualWriteRegister(device, AS726x_LED_CONTROL, value);
}

int AS726X_setIndicatorCurrent(AS726X_t *device, uint8_t current) {
    if (current > 0b11) current = 0b11;

    uint8_t value = virtualReadRegister(device, AS726x_LED_CONTROL);
    value &= 0b11111001;
    value |= (current << 1);
    return virtualWriteRegister(device, AS726x_LED_CONTROL, value);
}

int AS726X_enableBulb(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_LED_CONTROL);
    value |= (1 << 3);
    return virtualWriteRegister(device, AS726x_LED_CONTROL, value);
}

int AS726X_disableBulb(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_LED_CONTROL);
    value &= ~(1 << 3);
    return virtualWriteRegister(device, AS726x_LED_CONTROL, value);
}

int AS726X_setBulbCurrent(AS726X_t *device, uint8_t current) {
    if (current > 0b11) current = 0b11;

    uint8_t value = virtualReadRegister(device, AS726x_LED_CONTROL);
    value &= 0b11001111;
    value |= (current << 4);
    return virtualWriteRegister(device, AS726x_LED_CONTROL, value);
}

uint8_t AS726X_getTemperature(AS726X_t *device) {
    return virtualReadRegister(device, AS726x_DEVICE_TEMP);
}

float AS726X_getTemperatureF(AS726X_t *device) {
    float tempC = (float)AS726X_getTemperature(device);
    return tempC * 1.8f + 32.0f;
}

int AS726X_softReset(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_CONTROL_SETUP);
    value |= (1 << 7);
    return virtualWriteRegister(device, AS726x_CONTROL_SETUP, value);
}

// Funciones para obtener lecturas de colores

int AS726X_getViolet(AS726X_t *device) { return getChannel(device, AS7262_V); }
int AS726X_getBlue(AS726X_t *device) { return getChannel(device, AS7262_B); }
int AS726X_getGreen(AS726X_t *device) { return getChannel(device, AS7262_G); }
int AS726X_getYellow(AS726X_t *device) { return getChannel(device, AS7262_Y); }
int AS726X_getOrange(AS726X_t *device) { return getChannel(device, AS7262_O); }
int AS726X_getRed(AS726X_t *device) { return getChannel(device, AS7262_R); }

int AS726X_getR(AS726X_t *device) { return getChannel(device, AS7263_R); }
int AS726X_getS(AS726X_t *device) { return getChannel(device, AS7263_S); }
int AS726X_getT(AS726X_t *device) { return getChannel(device, AS7263_T); }
int AS726X_getU(AS726X_t *device) { return getChannel(device, AS7263_U); }
int AS726X_getV(AS726X_t *device) { return getChannel(device, AS7263_V); }
int AS726X_getW(AS726X_t *device) { return getChannel(device, AS7263_W); }

int AS726X_getX(AS726X_t *device) { return getChannel(device, AS7261_X); }
int AS726X_getY(AS726X_t *device) { return getChannel(device, AS7261_Y); }
int AS726X_getZ(AS726X_t *device) { return getChannel(device, AS7261_Z); }
int AS726X_getNIR(AS726X_t *device) { return getChannel(device, AS7261_NIR); }
int AS726X_getDark(AS726X_t *device) { return getChannel(device, AS7261_DARK); }
int AS726X_getClear(AS726X_t *device) { return getChannel(device, AS7261_CLEAR); }


float AS726X_getCalibratedViolet(AS726X_t *device) { return getCalibratedValue(device, AS7262_V_CAL); }
float AS726X_getCalibratedBlue(AS726X_t *device) { return getCalibratedValue(device, AS7262_B_CAL); }
float AS726X_getCalibratedGreen(AS726X_t *device) { return getCalibratedValue(device, AS7262_G_CAL); }
float AS726X_getCalibratedYellow(AS726X_t *device) { return getCalibratedValue(device, AS7262_Y_CAL); }
float AS726X_getCalibratedOrange(AS726X_t *device) { return getCalibratedValue(device, AS7262_O_CAL); }
float AS726X_getCalibratedRed(AS726X_t *device) { return getCalibratedValue(device, AS7262_R_CAL); }

float AS726X_getCalibratedR(AS726X_t *device) { return getCalibratedValue(device, AS7263_R_CAL); }
float AS726X_getCalibratedS(AS726X_t *device) { return getCalibratedValue(device, AS7263_S_CAL); }
float AS726X_getCalibratedT(AS726X_t *device) { return getCalibratedValue(device, AS7263_T_CAL); }
float AS726X_getCalibratedU(AS726X_t *device) { return getCalibratedValue(device, AS7263_U_CAL); }
float AS726X_getCalibratedV(AS726X_t *device) { return getCalibratedValue(device, AS7263_V_CAL); }
float AS726X_getCalibratedW(AS726X_t *device) { return getCalibratedValue(device, AS7263_W_CAL); }

float AS726X_getCalibratedX(AS726X_t *device) { return getCalibratedValue(device, AS7261_X_CAL); }
float AS726X_getCalibratedY(AS726X_t *device) { return getCalibratedValue(device, AS7261_Y_CAL); }
float AS726X_getCalibratedZ(AS726X_t *device) { return getCalibratedValue(device, AS7261_Z_CAL); }
float AS726X_getCalibratedX1931(AS726X_t *device) { return getCalibratedValue(device, AS7261_X1931_CAL); }
float AS726X_getCalibratedY1931(AS726X_t *device) { return getCalibratedValue(device, AS7261_Y1931_CAL); }
float AS726X_getCalibratedUPri1976(AS726X_t *device) { return getCalibratedValue(device, AS7261_UPRI_CAL); }
float AS726X_getCalibratedVPri1976(AS726X_t *device) { return getCalibratedValue(device, AS7261_VPRI_CAL); }
float AS726X_getCalibratedU1976(AS726X_t *device) { return getCalibratedValue(device, AS7261_U_CAL); }
float AS726X_getCalibratedV1976(AS726X_t *device) { return getCalibratedValue(device, AS7261_V_CAL); }
float AS726X_getCalibratedDUV1976(AS726X_t *device) { return getCalibratedValue(device, AS7261_DUV_CAL); }
int AS726X_getCalibratedLux(AS726X_t *device) { return getChannel(device, AS7261_LUX_CAL); }
int AS726X_getCalibratedCCT(AS726X_t *device) { return getChannel(device, AS7261_CCT_CAL); }


static int clearDataAvailable(AS726X_t *device) {
    uint8_t value = virtualReadRegister(device, AS726x_CONTROL_SETUP);
    value &= ~(1 << 1);
    return virtualWriteRegister(device, AS726x_CONTROL_SETUP, value);
}

static int getChannel(AS726X_t *device, uint8_t channelRegister) {
    int colorData = virtualReadRegister(device, channelRegister) << 8;
    colorData |= virtualReadRegister(device, channelRegister + 1);
    return colorData;
}

static float getCalibratedValue(AS726X_t *device, uint8_t calAddress) {
    uint8_t b0 = virtualReadRegister(device, calAddress + 0);
    uint8_t b1 = virtualReadRegister(device, calAddress + 1);
    uint8_t b2 = virtualReadRegister(device, calAddress + 2);
    uint8_t b3 = virtualReadRegister(device, calAddress + 3);

    uint32_t calBytes = 0;
    calBytes |= ((uint32_t)b0 << 24);
    calBytes |= ((uint32_t)b1 << 16);
    calBytes |= ((uint32_t)b2 << 8);
    calBytes |= ((uint32_t)b3);

    return convertBytesToFloat(calBytes);
}

static float convertBytesToFloat(uint32_t myLong) {
    float myFloat;
    memcpy(&myFloat, &myLong, sizeof(float));
    return myFloat;
}

static uint8_t virtualReadRegister(AS726X_t *device, uint8_t virtualAddr) {
    uint8_t status;
    uint8_t retries = 0;

    status = readRegister(device, AS72XX_SLAVE_STATUS_REG);
    if (status & AS72XX_SLAVE_RX_VALID) {
        readRegister(device, AS72XX_SLAVE_READ_REG);
    }

    while (1) {
        status = readRegister(device, AS72XX_SLAVE_STATUS_REG);
        if (!(status & AS72XX_SLAVE_TX_VALID)) break;
        vTaskDelay(pdMS_TO_TICKS(POLLING_DELAY));
        if (retries++ > MAX_RETRIES) return 0xFF;
    }

    if (writeRegister(device, AS72XX_SLAVE_WRITE_REG, virtualAddr) != 0) return 0xFF;

    retries = 0;

    while (1) {
        status = readRegister(device, AS72XX_SLAVE_STATUS_REG);
        if (status & AS72XX_SLAVE_RX_VALID) break;
        vTaskDelay(pdMS_TO_TICKS(POLLING_DELAY));
        if (retries++ > MAX_RETRIES) return 0xFF;
    }

    uint8_t incoming = readRegister(device, AS72XX_SLAVE_READ_REG);
    return incoming;
}

static int virtualWriteRegister(AS726X_t *device, uint8_t virtualAddr, uint8_t dataToWrite) {
    uint8_t status;
    uint8_t retries = 0;

    while (1) {
        status = readRegister(device, AS72XX_SLAVE_STATUS_REG);
        if (!(status & AS72XX_SLAVE_TX_VALID)) break;
        vTaskDelay(pdMS_TO_TICKS(POLLING_DELAY));
        if (retries++ > MAX_RETRIES) return -1;
    }

    if (writeRegister(device, AS72XX_SLAVE_WRITE_REG, virtualAddr | 0x80) != 0) return -1;

    retries = 0;

    while (1) {
        status = readRegister(device, AS72XX_SLAVE_STATUS_REG);
        if (!(status & AS72XX_SLAVE_TX_VALID)) break;
        vTaskDelay(pdMS_TO_TICKS(POLLING_DELAY));
        if (retries++ > MAX_RETRIES) return -1;
    }

    if (writeRegister(device, AS72XX_SLAVE_WRITE_REG, dataToWrite) != 0) return -1;

    return 0;
}

static uint8_t readRegister(AS726X_t *device, uint8_t addr) {
    uint8_t data = 0xFF;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(device->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo registro 0x%02X", addr);
        return 0xFF;
    }

    return data;
}

static int writeRegister(AS726X_t *device, uint8_t addr, uint8_t val) {
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(device->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error escribiendo registro 0x%02X", addr);
        return -1;
    }

    return 0;
}
