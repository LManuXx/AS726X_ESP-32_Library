/*
 * main.c
 * Use example of AS726X library for ESP32
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "AS726X.h"

static const char *TAG = "AS726X_Example";

#define I2C_MASTER_SCL_IO          22    // SCL pin of your ESP32
#define I2C_MASTER_SDA_IO          21    // SDA pin of your ESP32
#define I2C_MASTER_NUM             I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ         100000   
#define I2C_MASTER_TX_BUF_DISABLE  0          
#define I2C_MASTER_RX_BUF_DISABLE  0          
#define I2C_MASTER_TIMEOUT_MS      1000

void app_main(void)
{
    // Initialize I2C
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,         
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,     // Enable pull-up resistors
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,  // I2C frequency
        // .clk_flags = 0,                       // Only necesary for ESP32-S2 or superior hardware
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode,
                                       I2C_MASTER_RX_BUF_DISABLE,
                                       I2C_MASTER_TX_BUF_DISABLE, 0));

    // Initialize sensor
    AS726X_t sensor;
    if (AS726X_init(&sensor, i2c_num, AS726X_ADDR, 3, 3)) {
        ESP_LOGI(TAG, "Sensor AS726X inicializado correctamente");
    } else {
        ESP_LOGE(TAG, "Error al inicializar el sensor AS726X");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Get sensor version
    uint8_t version = AS726X_getVersion(&sensor);
    ESP_LOGI(TAG, "Versión del sensor: 0x%02X", version);


    while (1) {
        // take measurements
        if (AS726X_takeMeasurementsWithBulb(&sensor) != 0) {
            ESP_LOGE(TAG, "Error al tomar mediciones");
        } else {
            // read values
            int violet = AS726X_getViolet(&sensor);
            int blue = AS726X_getBlue(&sensor);
            int green = AS726X_getGreen(&sensor);
            int yellow = AS726X_getYellow(&sensor);
            int orange = AS726X_getOrange(&sensor);
            int red = AS726X_getRed(&sensor);

            // read calibrated values
            float calViolet = AS726X_getCalibratedViolet(&sensor);
            float calBlue = AS726X_getCalibratedBlue(&sensor);
            float calGreen = AS726X_getCalibratedGreen(&sensor);
            float calYellow = AS726X_getCalibratedYellow(&sensor);
            float calOrange = AS726X_getCalibratedOrange(&sensor);
            float calRed = AS726X_getCalibratedRed(&sensor);

            // show values
            ESP_LOGI(TAG, "Valores sin calibrar:");
            ESP_LOGI(TAG, "Violet: %d, Blue: %d, Green: %d, Yellow: %d, Orange: %d, Red: %d",
                     violet, blue, green, yellow, orange, red);

            ESP_LOGI(TAG, "Valores calibrados:");
            ESP_LOGI(TAG, "Violet: %.2f, Blue: %.2f, Green: %.2f, Yellow: %.2f, Orange: %.2f, Red: %.2f",
                     calViolet, calBlue, calGreen, calYellow, calOrange, calRed);

            // read temperature
            uint8_t tempC = AS726X_getTemperature(&sensor);
            float tempF = AS726X_getTemperatureF(&sensor);
            ESP_LOGI(TAG, "Temperatura: %d°C / %.2f°F", tempC, tempF);
        }

        // wait 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
