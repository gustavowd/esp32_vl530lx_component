/**
 * @file VL53L0X.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief C++ Library for VL53L0X as an ESP-IDF component
 * @copyright Copyright (c) 2018 Ryotaro Onuki
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
//#include "driver/i2c.h"
#include "driver/i2c_master.h"

#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

#include "esp_log.h"

static constexpr uint8_t VL53L0X_I2C_ADDRESS_DEFAULT = 0x29;

/**
 * @brief VL53L0X class
 *
 */
class VL53L0X {
public:
  VL53L0X(i2c_port_t i2c_port = I2C_NUM_0, gpio_num_t gpio_xshut = GPIO_NUM_MAX,
          gpio_num_t gpio_gpio1 = GPIO_NUM_MAX)
      : i2c_port(i2c_port), gpio_xshut(gpio_xshut), gpio_gpio1(gpio_gpio1) {
    vSemaphoreCreateBinary(xSemaphore);
  }

  bool init(i2c_master_dev_handle_t dev_handle, uint8_t device_addres = VL53L0X_I2C_ADDRESS_DEFAULT) {
    /* gpio init */
    ESP_LOGI(TAG, "Initing VL53L0X");
    if (gpio_xshut != GPIO_NUM_MAX) {
      gpio_set_direction(gpio_xshut, GPIO_MODE_OUTPUT);
      gpio_set_level(gpio_xshut, 1);
    }
    if (gpio_gpio1 != GPIO_NUM_MAX) {
      ESP_ERROR_CHECK(gpio_set_direction(gpio_gpio1, GPIO_MODE_INPUT));
      ESP_ERROR_CHECK(gpio_set_pull_mode(gpio_gpio1, GPIO_PULLUP_ONLY));
      ESP_ERROR_CHECK(gpio_pullup_en(gpio_gpio1));
      ESP_ERROR_CHECK(gpio_set_intr_type(gpio_gpio1, GPIO_INTR_POSEDGE));
      ESP_ERROR_CHECK(gpio_install_isr_service(0));
      ESP_ERROR_CHECK(gpio_isr_handler_add(gpio_gpio1, gpio1_isr, this));
      ESP_ERROR_CHECK(gpio_intr_enable(gpio_gpio1));
    }
    /* device init */
    vl53l0x_dev.dev_handle = dev_handle;
    vl53l0x_dev.i2c_port_num = i2c_port;
    vl53l0x_dev.i2c_address = device_addres;
    reset();
    if (init_vl53l0x(&vl53l0x_dev) != VL53L0X_ERROR_NONE)
      return false;
    if (VL53L0X_ERROR_NONE !=
        VL53L0X_SetGpioConfig(&vl53l0x_dev, 0,
                              VL53L0X_DEVICEMODE_SINGLE_RANGING,
                              VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
                              VL53L0X_INTERRUPTPOLARITY_LOW))
      return false;
    if (!setTimingBudget(33000))
      return false;
    return true;
  }
  bool reset() {
    if (!softwareReset())
      return hardwareReset();
    return true;
  }
  bool hardwareReset() {
    if (gpio_xshut == GPIO_NUM_MAX)
      return false;
    gpio_set_level(gpio_xshut, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(gpio_xshut, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    return true;
  }
  bool softwareReset() {
    VL53L0X_Error status = VL53L0X_ResetDevice(&vl53l0x_dev);
    ESP_LOGI(TAG, "VL53L0X reseted");
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_ResetDevice");
      return false;
    }
    return true;
  }
  /**
   * @brief Set the I2C address of the VL53L0X
   *
   * @param new_address right-aligned address
   */
  bool setDeviceAddress(uint8_t new_address) {
    VL53L0X_Error status =
        // VL53L0X_SetDeviceAddress expects the address to be left-aligned
        VL53L0X_SetDeviceAddress(&vl53l0x_dev, new_address << 1);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_PerformSingleRangingMeasurement");
      return false;
    }

    vl53l0x_dev.i2c_address = new_address;

    return true;
  }
  bool read(uint16_t *pRangeMilliMeter) {
    if (gpio_gpio1 != GPIO_NUM_MAX)
      return readSingleWithInterrupt(pRangeMilliMeter);
    return readSingleWithPolling(pRangeMilliMeter);
  }
  bool readSingleWithPolling(uint16_t *pRangeMilliMeter) {
    VL53L0X_RangingMeasurementData_t MeasurementData;
    VL53L0X_Error status =
        VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev, &MeasurementData);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_PerformSingleRangingMeasurement");
      return false;
    }
    *pRangeMilliMeter = MeasurementData.RangeMilliMeter;
    if (MeasurementData.RangeStatus != 0)
      return false;
    return true;
  }
  bool readSingleWithDelay(uint16_t *pRangeMilliMeter) {
    TickType_t xTicksToWait =
        TimingBudgetMicroSeconds / 1000 / portTICK_PERIOD_MS;
    VL53L0X_Error status;
    // set mode
    status =
        VL53L0X_SetDeviceMode(&vl53l0x_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_SetDeviceMode");
      return false;
    }
    // start measurement
    status = VL53L0X_StartMeasurement(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_StartMeasurement");
      return false;
    }
    // wait
    vTaskDelay(xTicksToWait);
    // get data
    VL53L0X_RangingMeasurementData_t MeasurementData;
    status = VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &MeasurementData);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_GetRangingMeasurementData");
      return false;
    }
    *pRangeMilliMeter = MeasurementData.RangeMilliMeter;
    if (MeasurementData.RangeStatus != 0)
      return false;
    // clear interrupt
    VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);
    if (status != VL53L0X_ERROR_NONE)
      return false;
    return true;
  }
  bool readSingleWithInterrupt(uint16_t *pRangeMilliMeter) {
    if (gpio_gpio1 == GPIO_NUM_MAX)
      return false;
    VL53L0X_Error status;
    // set mode
    status =
        VL53L0X_SetDeviceMode(&vl53l0x_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_SetDeviceMode");
      return false;
    }
    // clear semphr
    xSemaphoreTake(xSemaphore, 0);
    // start measurement
    status = VL53L0X_StartMeasurement(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_StartMeasurement");
      return false;
    }
    // wait for interrupt
    xSemaphoreTake(xSemaphore, 1000 / portMAX_DELAY);
    // get data
    VL53L0X_RangingMeasurementData_t MeasurementData;
    status = VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &MeasurementData);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_GetRangingMeasurementData");
      return false;
    }
    *pRangeMilliMeter = MeasurementData.RangeMilliMeter;
    if (MeasurementData.RangeStatus != 0)
      return false;
    // clear interrupt
    VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);
    if (status != VL53L0X_ERROR_NONE)
      return false;
    return true;
  }

    bool readContinuousWithInterrupt(uint16_t *pRangeMilliMeter) {
    if (gpio_gpio1 == GPIO_NUM_MAX)
      return false;
    VL53L0X_Error status;
    // set mode
    status =
        VL53L0X_SetDeviceMode(&vl53l0x_dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_SetDeviceMode");
      return false;
    }
    // clear semphr
    xSemaphoreTake(xSemaphore, 0);
    // start measurement
    status = VL53L0X_StartMeasurement(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_StartMeasurement");
      return false;
    }
    // wait for interrupt
    xSemaphoreTake(xSemaphore, 1000 / portMAX_DELAY);
    // get data
    VL53L0X_RangingMeasurementData_t MeasurementData;
    status = VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &MeasurementData);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_GetRangingMeasurementData");
      return false;
    }
    *pRangeMilliMeter = MeasurementData.RangeMilliMeter;
    if (MeasurementData.RangeStatus != 0)
      return false;
    // clear interrupt
    VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);
    if (status != VL53L0X_ERROR_NONE)
      return false;
    return true;
  }


  void i2cMasterInit(i2c_master_bus_handle_t *bus_handle,
                    i2c_master_dev_handle_t *dev_handle,
                    i2c_port_t i2c_port = I2C_NUM_0,
                    gpio_num_t pin_sda = GPIO_NUM_21,
                    gpio_num_t pin_scl = GPIO_NUM_22, uint32_t freq = 400000, uint8_t device_addres = VL53L0X_I2C_ADDRESS_DEFAULT) {

    i2c_master_bus_config_t bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = pin_sda,
        .scl_io_num = pin_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,  /* 0 = synchronous mode */
        .flags = {.enable_internal_pullup = true, .allow_pd = false}
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
    
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addres,
        .scl_speed_hz = freq,
        .scl_wait_us = 0,
        .flags ={.disable_ack_check = true}
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle)); 
    ESP_LOGI(TAG, "I2C initialized: SDA=%d, SCL=%d, Freq=%lu Hz", pin_sda, pin_scl, freq);
  }

  bool setTimingBudget(uint32_t TimingBudgetMicroSeconds) {
    VL53L0X_Error status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
        &vl53l0x_dev, TimingBudgetMicroSeconds);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
      return false;
    }
    this->TimingBudgetMicroSeconds = TimingBudgetMicroSeconds;
    return true;
  }

  bool setVcselPulsePeriod(VL53L0X_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriod) {
    VL53L0X_Error status = VL53L0X_SetVcselPulsePeriod(&vl53l0x_dev, VcselPeriodType, VCSELPulsePeriod);
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_setVcselPulsePeriod");
      return false;
    }
    return true;
  }

  // Set the return signal rate limit check value in units of MCPS (mega counts
  // per second). "This represents the amplitude of the signal reflected from the
  // target and detected by the device"; setting this limit presumably determines
  // the minimum measurement necessary for the sensor to report a valid reading.
  // Setting a lower limit increases the potential range of the sensor but also
  // seems to increase the likelihood of getting an inaccurate reading because of
  // unwanted reflections from objects other than the intended target.
  // Defaults to 0.25 MCPS as initialized by the ST API and this library.
  bool setSignalRateLimit(float limit_Mcps)
  {
    if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    VL53L0X_Error status = VL53L0X_WrWord(&vl53l0x_dev, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
    if (status != VL53L0X_ERROR_NONE) {
      print_pal_error(status, "VL53L0X_setSignalRateLimit");
      return false;
    }
    return true;
  }

protected:
  static constexpr const char *TAG = "VL53L0X";
  i2c_port_t i2c_port;
  gpio_num_t gpio_xshut;
  gpio_num_t gpio_gpio1;
  VL53L0X_Dev_t vl53l0x_dev;
  volatile SemaphoreHandle_t xSemaphore = NULL;
  int32_t TimingBudgetMicroSeconds;

  static void IRAM_ATTR gpio1_isr(void *arg) {
    VL53L0X *obj = static_cast<VL53L0X *>(arg);
    xSemaphoreGiveFromISR(obj->xSemaphore, NULL);
  }
  static VL53L0X_Error print_pal_error(VL53L0X_Error status,
                                       const char *method) {
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(status, buf);
    ESP_LOGE(TAG, "%s API status: %i : %s\n", method, status, buf);
    return status;
  }
  static VL53L0X_Error init_vl53l0x(VL53L0X_Dev_t *pDevice) {
    VL53L0X_Error status;
    uint8_t isApertureSpads;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t VhvSettings;
    // Device Initialization (~40ms)
    status = VL53L0X_DataInit(pDevice);
    if (status != VL53L0X_ERROR_NONE)
      return print_pal_error(status, "VL53L0X_DataInit");
    status = VL53L0X_StaticInit(pDevice);
    if (status != VL53L0X_ERROR_NONE)
      return print_pal_error(status, "VL53L0X_StaticInit");
    // SPADs calibration (~10ms)
    status = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount,
                                              &isApertureSpads);
    ESP_LOGI(TAG, "refSpadCount = %" PRIu32 ", isApertureSpads = %" PRIu8 "\n",
             refSpadCount, isApertureSpads);
    if (status != VL53L0X_ERROR_NONE)
      return print_pal_error(status, "VL53L0X_PerformRefSpadManagement");
    // Temperature calibration (~40ms)
    status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal);
    if (status != VL53L0X_ERROR_NONE)
      return print_pal_error(status, "VL53L0X_PerformRefCalibration");
    // Setup in single ranging mode
    status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE)
      return print_pal_error(status, "VL53L0X_SetDeviceMode");
    // end
    return status;
  }
};
