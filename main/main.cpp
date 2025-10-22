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
#define INT_PIN   GPIO_NUM_32
#define CI1_IO_PIN GPIO_NUM_25
#define MASTER_FREQUENCY 400000
#define PORT_NUMBER I2C_NUM_0

#define INT2_PIN   GPIO_NUM_33
#define CI2_IO_PIN GPIO_NUM_26

#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define LENGTH 48
static const char *TAG = "VL530LX";

#define VL53L0X_DUAL_SENSOR          1
#define HIGH_SPEED
//#define HIGH_ACCURACY
//#define LONG_RANGE


extern "C" void app_main(void)
{
  static i2c_master_bus_handle_t bus_handle;
  static i2c_master_dev_handle_t dev_handle;

  VL53L0X vl(PORT_NUMBER, CI1_IO_PIN);
  #if (VL53L0X_DUAL_SENSOR == 1)
  VL53L0X vl2(PORT_NUMBER, CI2_IO_PIN, INT2_PIN);
  #endif
  //ESP_ERROR_CHECK(gpio_install_isr_service(0));
  vl.i2cMasterInit(&bus_handle, &dev_handle, PORT_NUMBER, SDA_IO_PIN, SCL_IO_PIN, 400000, VL53L0X_I2C_ADDRESS_DEFAULT);
  #if (VL53L0X_DUAL_SENSOR == 1)
  if (!vl.init(dev_handle, 22)) {
    ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
    vTaskDelay(portMAX_DELAY);
  }
  #else
  if (!vl.init(dev_handle, 22)) {
    ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
    vTaskDelay(portMAX_DELAY);
  }
  #endif

  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  vl.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
	vl.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	vl.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
	vl.setTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  vl.setTimingBudget(200000);
#endif

  #if (VL53L0X_DUAL_SENSOR == 1)
  if (!vl2.init(dev_handle, 25)) {
    ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
    vTaskDelay(portMAX_DELAY);
  }
  //vl2.setTimingBudget(300 * 1000UL);
  #endif

  while (1) {
    /* measurement */
    uint16_t result_mm = 0;
    TickType_t tick_start = xTaskGetTickCount();
    bool res = vl.read(&result_mm);
    //bool res = vl.readSingleWithInterrupt(&result_mm);
    TickType_t tick_end = xTaskGetTickCount();

    int took_ms = ((int)tick_end - tick_start);
    if (res)
      ESP_LOGI(TAG, "Range sensor 1: %d [mm] took %d [ms]", (int)result_mm, took_ms);
    else
      ESP_LOGE(TAG, "Failed to measure :(");

    #if (VL53L0X_DUAL_SENSOR == 1)
    uint16_t result_mm2 = 0;
    TickType_t tick_start2 = xTaskGetTickCount();
    bool res2 = vl2.readSingleWithInterrupt(&result_mm2);
    TickType_t tick_end2 = xTaskGetTickCount();

    int took_ms2 = ((int)tick_end2 - tick_start2) / portTICK_PERIOD_MS;
    if (res2)
      ESP_LOGI(TAG, "Range sensor 2: %d [mm] took %d [ms]", (int)result_mm2, took_ms2);
    else
      ESP_LOGE(TAG, "Failed to measure :(");
    #endif

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
