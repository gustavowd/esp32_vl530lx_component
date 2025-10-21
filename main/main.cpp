/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "VL53L0X.h"

#define SCL_IO_PIN GPIO_NUM_22
#define SDA_IO_PIN GPIO_NUM_21
#define INT_PIN   GPIO_NUM_19
#define CI1_IO_PIN GPIO_NUM_25
#define MASTER_FREQUENCY 400000
#define PORT_NUMBER I2C_NUM_0

#define SCL2_IO_PIN GPIO_NUM_4
#define SDA2_IO_PIN GPIO_NUM_5
#define INT2_PIN   GPIO_NUM_6
#define CI2_IO_PIN GPIO_NUM_26
#define MASTER_FREQUENCY 400000
#define PORT2_NUMBER I2C_NUM_1

#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define LENGTH 48
static const char *TAG = "VL530LX";


extern "C" void app_main(void)
{
  /*
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL<<CI1_IO_PIN) | (1ULL<<CI2_IO_PIN));
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

  gpio_set_level(CI1_IO_PIN, 0);
  gpio_set_level(CI2_IO_PIN, 1);
  */
  static i2c_master_bus_handle_t bus_handle;
  static i2c_master_dev_handle_t dev_handle;

    static i2c_master_bus_handle_t bus_handle2;
  static i2c_master_dev_handle_t dev_handle2;

  VL53L0X vl(PORT_NUMBER, CI1_IO_PIN, INT_PIN);
  vl.i2cMasterInit(&bus_handle, &dev_handle, PORT_NUMBER, SDA_IO_PIN, SCL_IO_PIN, 400000, VL53L0X_I2C_ADDRESS_DEFAULT);
  if (!vl.init(dev_handle, VL53L0X_I2C_ADDRESS_DEFAULT)) {
    ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
    vTaskDelay(portMAX_DELAY);
  }

  VL53L0X vl2(PORT2_NUMBER, CI2_IO_PIN, INT2_PIN);
  vl2.i2cMasterInit(&bus_handle2, &dev_handle2, PORT2_NUMBER, SDA2_IO_PIN, SCL2_IO_PIN, 400000, VL53L0X_I2C_ADDRESS_DEFAULT);
  if (!vl2.init(dev_handle, VL53L0X_I2C_ADDRESS_DEFAULT)) {
    ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
    vTaskDelay(portMAX_DELAY);
  }

  	// Configure the sensor for high accuracy and speed in 20 cm.
	vl.setSignalRateLimit(200);
	vl.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 10);
	vl.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

  // Configurar timing budget para medições mais rápidas (33ms)
	vl.setTimingBudget(300 * 1000UL);
  vl2.setTimingBudget(300 * 1000UL);

  while (1) {
    /* measurement */
    uint16_t result_mm = 0;
    uint16_t result_mm2 = 0;
    TickType_t tick_start = xTaskGetTickCount();
    //bool res = vl.read(&result_mm);
    bool res = vl.readSingleWithInterrupt(&result_mm);
    TickType_t tick_end = xTaskGetTickCount();

    TickType_t tick_start2 = xTaskGetTickCount();
    bool res2 = vl2.readSingleWithInterrupt(&result_mm2);
    TickType_t tick_end2 = xTaskGetTickCount();
    
    int took_ms = ((int)tick_end - tick_start) / portTICK_PERIOD_MS;
    if (res)
      ESP_LOGI(TAG, "Range sensor 1: %d [mm] took %d [ms]", (int)result_mm, took_ms);
    else
      ESP_LOGE(TAG, "Failed to measure :(");

    int took_ms2 = ((int)tick_end2 - tick_start2) / portTICK_PERIOD_MS;
    if (res2)
      ESP_LOGI(TAG, "Range sensor 2: %d [mm] took %d [ms]", (int)result_mm2, took_ms2);
    else
      ESP_LOGE(TAG, "Failed to measure :(");
  }
}
