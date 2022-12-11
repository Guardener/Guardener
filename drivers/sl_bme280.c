/*******************************************************************************
 * @file sl_bme280.C
 *
 *  Created on: Nov 16, 2022
 *      Author: Caleb Provost
 *
 * @brief
 *      Wrapper code to implement Bosch's BME280: Digital humidity, pressure, and temperature sensor
 * @see
 *      www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
 *      https://github.com/BoschSensortec/BME280_driver
 *
 * @note  Code structure created following existing SiLabs driver methodology
 *
 ******************************************************************************/

#include "sl_bme280.h"
#include "bme280.h"
#include "sl_sleeptimer.h"
#include "app_log.h"
#include <stdbool.h>
#include <math.h>

#ifdef app_log_debug
#define DLOGRET(...)                                                                                                   \
    do {                                                                                                               \
        app_log_debug(__VA_ARGS__);                                                                                    \
        app_log_nl();                                                                                                  \
    } while (0)
#else
#define DLOGRET(...) do { /* nop */ } while (0)
#define app_log_debug(...) do { /* nop */ } while (0)
#define app_log_info(...) do { /* nop */ } while (0)
#define app_log_error(...) do { /* nop */ } while (0)
#define app_log_nl(...) do { /* nop */ } while (0)
#endif

// For relative humidity adjusting/calibrating
#define OFFSET 58
#define SCALING 0.00454

static bool bme280_initialized_s = false, bme280_asleep = false;
static struct bme280_dev dev_s = {0};
static uint8_t op_mode_s;

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the
 * input provided in some of the APIs
 */
void sl_bme280_delay_us(uint32_t period, void *intf_ptr)
{
    // ignore compiler warnings of unused parameter
    sl_i2cspm_t* i2cspm = (sl_i2cspm_t* )intf_ptr;
    intf_ptr = i2cspm;
    sl_sleeptimer_delay_millisecond(period / 1000);
}

/*!
 * @brief This function reading the sensor's registers through I2C bus.
 */
int8_t sl_bme280_read_register(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    sl_i2cspm_t* i2cspm = (sl_i2cspm_t* )intf_ptr;
    int8_t ret = BME280_OK;
    I2C_TransferSeq_TypeDef seq;

    seq.addr = dev_s.chip_id << 1;
    seq.flags = I2C_FLAG_WRITE_READ;

    /* Select register to start reading from */
    seq.buf[0].data = &reg_addr;
    seq.buf[0].len = 1;

    /* Select length of data to be read */
    seq.buf[1].data = data;
    seq.buf[1].len = len;

    I2C_TransferReturn_TypeDef retval = I2CSPM_Transfer(i2cspm, &seq);
    if (retval != i2cTransferDone)
    {
        ret = BME280_E_COMM_FAIL;
    }

    return ret;
}

/*!
 * @brief This function for writing the sensor's registers through I2C bus.
 */
int8_t sl_bme280_write_register(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    sl_i2cspm_t* i2cspm = (sl_i2cspm_t* )intf_ptr;
    int8_t ret = BME280_OK;
    I2C_TransferSeq_TypeDef seq;
    uint8_t i2c_write_data[len*2];
    uint8_t i2c_read_data[1];

    seq.addr = dev_s.chip_id << 1;
    seq.flags = I2C_FLAG_WRITE;

    /* Select register to start writing to*/
    i2c_write_data[0] = reg_addr;
    for(uint8_t i = 0; i < len; i++)
    {
        i2c_write_data[i + 1] = data[i];
    }
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len = len + 1;
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len = 0;

    I2C_TransferReturn_TypeDef retval = I2CSPM_Transfer(i2cspm, &seq);
    if (retval != i2cTransferDone)
    {
        ret = BME280_E_COMM_FAIL;
    }

    return ret;
}

uint8_t sl_bme280_convert_bme2RH(float humi)
{
    uint8_t scaled_humid = -1; // not a possible return value. Acts as an error

    // Amber: I came up with (0.13303 + 0.02017 x) / 0.653182 - 0.0612587 x)
    #if !APP_LOG_LEVEL_MASK_DEBUG
    app_log_debug("RAW DATA BME280 acquired: %0.2lf", humi);
    app_log_nl();
    #endif
    #if !APP_LOG_LEVEL_MASK_DEBUG
    app_log_debug("OFFSET/SCALED DATA BME280 acquired: %0.2lf", (humi - OFFSET) * 1000);
    app_log_nl();
    #endif

    const float a = 0.5556, b = 1.85;
    scaled_humid = (a * log( b - (humi - OFFSET) )) * 100;

    #if !APP_LOG_LEVEL_MASK_DEBUG
    app_log_debug("BME280 humidity scaled to %d%% RH", scaled_humid);
    app_log_nl();
    #endif

    return (scaled_humid > 100) ? -1 /* failure */ : scaled_humid;
}

sl_status_t sl_bme280_force_get_readings(float *temp, float* pres, float* humi)
{
    if (!bme280_initialized_s) {
        return SL_STATUS_NOT_INITIALIZED;
    }

    /**
     * Calculate the minimum delay required between consecutive measurement based upon
     * the sensor enabled and the oversampling configuration
     */
    uint32_t req_delay = bme280_cal_meas_delay(&dev_s.settings);

    // Wake up device if it's asleep
    bool was_asleep = false;
    if (bme280_asleep) {
        was_asleep = true;
        // Also triggers the BME280 to acquire readings
        if (sl_bme280_wake_up_device() != SL_STATUS_OK) {
            return SL_STATUS_FAIL;
        }
        // Wait for the measurement to complete
        sl_bme280_delay_us(req_delay * 1000, NULL);
    }

    struct bme280_data comp_data;
    int8_t ret = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev_s);
    if (ret != BME280_OK) {
        app_log_error("Failed to get sensor data (code %+d).", ret);
        app_log_nl();
        return SL_STATUS_FAIL;
    }

#ifdef BME280_FLOAT_ENABLE
    if (temp != NULL) {
        *temp = (float)comp_data.temperature;
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("BME280 acquired %0.2lf deg C", *temp);
        app_log_nl();
        #endif
    }
    if (pres != NULL) {
        *pres = (float)0.01 * comp_data.pressure;
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("BME280 acquired %0.2lf hPa", *pres);
        app_log_nl();
        #endif
    }
    if (humi != NULL) {
        *humi = (float)comp_data.humidity;
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("BME280 acquired %0.2lf%%", *humi);
        app_log_nl();
        #endif
    }
#elif BME280_64BIT_ENABLE
    if (temp != NULL) {
        *temp = (float)0.01f * comp_data.temperature;
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("BME280 acquired %0.2lf deg C", *temp);
        app_log_nl();
        #endif
    }
    if (pres != NULL) {
        *pres = (float)0.0001f * comp_data.pressure;
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("BME280 acquired %0.2lf hPa", *pres);
        app_log_nl();
        #endif
    }
    if (humi != NULL) {
        *humi = (float)1.0f / 1024.0f * comp_data.humidity;
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("BME280 acquired %0.2lf%%", *humi);
        app_log_nl();
        #endif
    }
#else
    if (temp != NULL) {
        *temp = (float)0.01f * comp_data.temperature;
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("BME280 acquired %0.2lf deg C", *temp);
        app_log_nl();
        #endif
    }
    if (pres != NULL) {
        *pres = (float)0.01f * comp_data.pressure;
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("BME280 acquired %0.2lf hPa", *pres);
        app_log_nl();
        #endif
    }
    if (humi != NULL) {
        *humi = (float)1.0f / 1024.0f * comp_data.humidity;
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("BME280 acquired %0.2lf%%", *humi);
        app_log_nl();
        #endif
    }
#endif

    // if device was sleeping, return to sleep mode
    if (was_asleep) {
        return sl_bme280_put_to_sleep();
    }

    return SL_STATUS_OK;
}

sl_status_t sl_bme280_wake_up_device()
{
    if (!bme280_initialized_s) {
        return SL_STATUS_NOT_INITIALIZED;
    }

    if (!bme280_asleep) {
        return SL_STATUS_OK;
    }
    
    if (bme280_set_sensor_mode(op_mode_s, &dev_s) != BME280_OK) {
        return SL_STATUS_FAIL;
    } else {
        bme280_asleep = false;
    }

    return SL_STATUS_OK;
}

sl_status_t sl_bme280_put_to_sleep()
{
    if (!bme280_initialized_s) {
        return SL_STATUS_NOT_INITIALIZED;
    }

    if (bme280_asleep) {
        return SL_STATUS_OK;
    }
    
    if (bme280_set_sensor_mode(BME280_SLEEP_MODE, &dev_s) != BME280_OK) {
        return SL_STATUS_FAIL;
    } else {
        bme280_asleep = true;
    }

    return SL_STATUS_OK;
}

/*!
 * @brief Initializes the BME280 with SiLab source code
 */
sl_status_t sl_bme280_init(sl_i2cspm_t* i2cspm, struct bme280_settings cfg, uint8_t mode)
{
    int8_t ret = BME280_OK;

    if (!i2cspm) {
        return SL_STATUS_INVALID_HANDLE;
    }

    if (mode != (BME280_SLEEP_MODE || BME280_FORCED_MODE || BME280_NORMAL_MODE)) {
        return SL_STATUS_INVALID_MODE;
    } else {
        op_mode_s = mode;
    }

    struct bme280_dev dev = {
        .chip_id = BME280_I2C_ADDR_PRIM,    /*< Chip Id */
        .intf_ptr = i2cspm,                 /*< Interface pointer to the device address I2C */
        .intf = BME280_I2C_INTF,            /*< For I2C, intf = BME280_I2C_INTF */
        .read = sl_bme280_read_register,    /*< Read function pointer */
        .write = sl_bme280_write_register,  /*< Write function pointer */
        .delay_us = sl_bme280_delay_us,     /*< Delay function pointer */
        .calib_data = {0},                  /*< Trim data */
        .settings = cfg,                    /*< Sensor settings */
        .intf_rslt = ret                    /*< Variable to store result of read/write function */
    };
    dev_s = dev; // Static variable used during init process, needs to be copied before init process

    if (bme280_init(&dev) != BME280_OK) {
        return SL_STATUS_FAIL;
    }
    dev_s.calib_data = dev.calib_data; // initialization process updates calibration data


    uint8_t settings_sel = BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL;
    ret = bme280_set_sensor_settings(settings_sel, &dev_s);
    if (ret != BME280_OK) {
        app_log_error("Failed to set sensor settings (code %+d).", ret);
        app_log_nl();
        return SL_STATUS_NOT_INITIALIZED;
    }

    ret = bme280_set_sensor_mode(op_mode_s, &dev_s);
    if (ret != BME280_OK) {
        app_log_error("Failed to set sensor mode (code %+d).", ret);
        app_log_nl();
        return SL_STATUS_INVALID_MODE;
    }

    bme280_initialized_s = true;
    if (sl_bme280_put_to_sleep() == SL_STATUS_OK) {
        bme280_asleep = true;
    }

    return SL_STATUS_OK;
}
