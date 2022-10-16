#include "bme280.h"
#include "app_log.h"
#include "cmsis_gcc.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define DLOGRET(...)                                                                                                   \
    do {                                                                                                               \
        app_log_debug(__VA_ARGS__);                                                                                    \
        sl_status_print(ret);                                                                                          \
    } while (0)

union data {
    uint8_t data8[4];
    uint16_t data16[2];
};
static sl_i2cspm_t *i2cspm_handle_s = NULL;
static bme280_calib_t calib_data_s;
static bool forced_mode_s;
static uint8_t forced_cfg_s;
static int32_t t_fine;

/***************************************************************************//**
 *    Reads register from the BME280 sensor
 ******************************************************************************/
sl_status_t bme280_read_register(sl_i2cspm_t* i2cspm, uint8_t reg, uint8_t* data)
{
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef retval;
    uint8_t i2c_write_data[1];
    sl_status_t ret = SL_STATUS_OK;

    seq.addr = BME280_I2C_DEVICE_BUS_ADDRESS << 1;
    seq.flags = I2C_FLAG_WRITE_READ;

    /* Select register to start reading from */
    i2c_write_data[0] = reg;
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len = 1;

    /* Select length of data to be read */
    seq.buf[1].data = data;
    seq.buf[1].len = 1;

    retval = I2CSPM_Transfer(i2cspm, &seq);
    if(retval != i2cTransferDone)
    {
        *data = 0xff;
        ret = SL_STATUS_TRANSMIT;
    }

    return ret;
}

/***************************************************************************//**
 *    Reads a block of data from the BME280 sensor.
 ******************************************************************************/
sl_status_t bme280_read_register_block(sl_i2cspm_t* i2cspm, uint8_t reg, uint8_t length, uint8_t* data)
{
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef retval;
    uint8_t i2c_write_data[1];
    sl_status_t ret = SL_STATUS_OK;

    seq.addr = BME280_I2C_DEVICE_BUS_ADDRESS << 1;
    seq.flags = I2C_FLAG_WRITE_READ;

    /* Select register to start reading from */
    i2c_write_data[0] = reg;
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len = 1;

    /* Select length of data to be read */
    seq.buf[1].data = data;
    seq.buf[1].len = length;

    retval = I2CSPM_Transfer(i2cspm, &seq);
    if(retval != i2cTransferDone)
    {
        ret = SL_STATUS_TRANSMIT;
    }

    return ret;
}

/***************************************************************************//**
 *    Writes register in the BME280 sensor
 ******************************************************************************/
sl_status_t bme280_write_register(sl_i2cspm_t* i2cspm, uint8_t reg, uint8_t data)
{
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef retval;
    uint8_t i2c_write_data[4];
    uint8_t i2c_read_data[1];
    sl_status_t ret = SL_STATUS_OK;

    seq.addr = BME280_I2C_DEVICE_BUS_ADDRESS << 1;
    seq.flags = I2C_FLAG_WRITE;

    /* Select register and data to write */
    i2c_write_data[0] = reg;
    i2c_write_data[1] = data;
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len = 2;
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len = 0;

    retval = I2CSPM_Transfer(i2cspm, &seq);
    if(retval != i2cTransferDone)
    {
        ret = SL_STATUS_TRANSMIT;
    }

    return ret;
}

/***************************************************************************//**
 *    Writes a block of data to the BME280 sensor.
 ******************************************************************************/
sl_status_t bme280_write_register_block(sl_i2cspm_t* i2cspm, uint8_t reg, uint8_t length, const uint8_t* data)
{
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef retval;
    uint8_t i2c_write_data[10];
    uint8_t i2c_read_data[1];
    uint8_t i;
    sl_status_t ret = SL_STATUS_OK;

    seq.addr = BME280_I2C_DEVICE_BUS_ADDRESS << 1;
    seq.flags = I2C_FLAG_WRITE;

    /* Select register to start writing to*/
    i2c_write_data[0] = reg;
    for(i = 0; i < length; i++)
    {
        i2c_write_data[i + 1] = data[i];
    }
    seq.buf[0].data = i2c_write_data;
    seq.buf[0].len = length + 1;
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len = 0;

    retval = I2CSPM_Transfer(i2cspm, &seq);
    if(retval != i2cTransferDone)
    {
        ret = SL_STATUS_TRANSMIT;
    }

    return ret;
}

bool bme280_is_calibrating(void)
{
    uint8_t data = 0x00;
    bme280_read_register(i2cspm_handle_s, BME280_REG_STATUS, &data);
    return BME280_UPDATING_CHECK(data) != 0;
}

sl_status_t bme280_get_compensation_params(void)
{
    sl_status_t ret = SL_STATUS_OK;
//    uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
    union data d;

    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB00, 2, d.data8);
    calib_data_s.dig_T1 = __REV16(d.data16[0]);
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB02, 2, d.data8);
    calib_data_s.dig_T2 = __REVSH(d.data16[0]);
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB04, 2, d.data8);
    calib_data_s.dig_T3 = __REVSH(d.data16[0]);
    DLOGRET("Aquiring Temperature Calibration Data");

    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB06, 2, d.data8);
    calib_data_s.dig_P1 = __REV16(d.data16[0]);
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB08, 2, d.data8);
    calib_data_s.dig_P2 = __REVSH(d.data16[0]);
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB10, 2, d.data8);
    calib_data_s.dig_P3 = __REVSH(d.data16[0]);
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB12, 2, d.data8);
    calib_data_s.dig_P4 = __REVSH(d.data16[0]);
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB14, 2, d.data8);
    calib_data_s.dig_P5 = __REVSH(d.data16[0]);
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB16, 2, d.data8);
    calib_data_s.dig_P6 = __REVSH(d.data16[0]);
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB18, 2, d.data8);
    calib_data_s.dig_P7 = __REVSH(d.data16[0]);
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB20, 2, d.data8);
    calib_data_s.dig_P8 = __REVSH(d.data16[0]);
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB22, 2, d.data8);
    calib_data_s.dig_P9 = __REVSH(d.data16[0]);
    DLOGRET("Aquiring Pressure Calibration Data");

    ret += bme280_read_register(i2cspm_handle_s, BME280_REG_CALIB25, d.data8);
    calib_data_s.dig_H1 = d.data8[0];
    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_CALIB26, 2, d.data8);
    calib_data_s.dig_H2 = __REVSH(d.data16[0]);
    ret += bme280_read_register(i2cspm_handle_s, BME280_REG_CALIB28, d.data8);
    calib_data_s.dig_H3 = d.data8[0];
    ret += bme280_read_register(i2cspm_handle_s, BME280_REG_CALIB29, d.data8);
    uint8_t tmp = 0x00;
    ret += bme280_read_register(i2cspm_handle_s, BME280_REG_CALIB30, &tmp);
    calib_data_s.dig_H4 = ((int8_t)d.data8[0] << 4) | (tmp & 0x0F);
    ret += bme280_read_register(i2cspm_handle_s, BME280_REG_CALIB31, d.data8);
    ret += bme280_read_register(i2cspm_handle_s, BME280_REG_CALIB30, &tmp);
    calib_data_s.dig_H5 = ((int8_t)d.data8[0] << 4) | (tmp >> 4);
    ret += bme280_read_register(i2cspm_handle_s, BME280_REG_CALIB32, d.data8);
    calib_data_s.dig_H6 = (int8_t)d.data8[0];
    DLOGRET("Aquiring Humidity Calibration Data");

    return ret;
}

sl_status_t bme280_init(sl_i2cspm_t* i2cspm, bme280_init_t init)
{
    app_log_debug("Temp/RH/Pressure Sensor Initialization\n");
    sl_status_t ret = SL_STATUS_OK;
    if(!i2cspm)
    {
        return SL_STATUS_NULL_POINTER;
    }
    i2cspm_handle_s = i2cspm;
    uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};

    // check if sensor, i.e. the chip ID is correct
    ret += bme280_read_register(i2cspm_handle_s, BME280_REG_ID, data);
    if(data[0] != BME280_CHIP_ID)
    {
        app_log_error("BME280's Chip ID doesn't match expected!");
        return SL_STATUS_INVALID_CONFIGURATION;
    }

    // Wake BME280
    ret += bme280_write_register(i2cspm_handle_s, BME280_REG_RESET, BME280_CMD_RESET_POWER_ON);
    sl_sleeptimer_delay_millisecond(10);

    // Wait longer if still calibrating
    while(bme280_is_calibrating())
    {
        sl_sleeptimer_delay_millisecond(10);
    }

    // Get the calibrated data
    ret += bme280_get_compensation_params();

    // Putting the BME280 into sleep mode ensures configuration commands won't be ignored
    ret += bme280_write_register(i2cspm_handle_s, BME280_REG_CTRL_MEAS, BEM280_CMD_CTRL_MEAS_MODE_SLEEP);

    // Track which operating mode the BME280 is configured with
    forced_mode_s = (init.opt_mode == BEM280_CMD_CTRL_MEAS_MODE_FORCED) ? true : false;
    forced_cfg_s = forced_mode_s ? init.opt_mode | init.temp_samp_rate | init.pres_samp_rate : 0xFF;

    // Changes to this register only become effective after a write operation to "ctrl_meas".
    ret += bme280_write_register(i2cspm_handle_s, BME280_REG_CTRL_HUM, init.hum_samp_rate);
    ret += bme280_write_register(i2cspm_handle_s, BME280_REG_CONFIG,
    BME280_REG_CONFIG_I2C | init.acqu_intvl | init.filter);
    ret += bme280_write_register(i2cspm_handle_s, BME280_REG_CTRL_MEAS,
                                 init.opt_mode | init.temp_samp_rate | init.pres_samp_rate);

    // sl_sleeptimer_delay_millisecond(100);

    return ret;
}

sl_status_t bme280_deinit(void)
{
    sl_status_t ret = SL_STATUS_OK;
    ret += bme280_write_register(i2cspm_handle_s, BME280_REG_CTRL_MEAS, BEM280_CMD_CTRL_MEAS_MODE_SLEEP);
    if(ret != SL_STATUS_OK)
    {
        return ret;
    }
    i2cspm_handle_s = NULL;
    memset(&calib_data_s, 0x00, sizeof(bme280_calib_t));
    return ret;
}

// Forces the BME280 to take a measurement which then goes back to sleep mode
sl_status_t bme280_force_trigger_measurements(void)
{
    sl_status_t ret = SL_STATUS_INVALID_CONFIGURATION;
    uint8_t data;
    if(forced_mode_s)
    {
        bool once = false;
        uint32_t sleep_iters = 20;
        ret = SL_STATUS_OK;
        // Get measurement
        ret += bme280_write_register(i2cspm_handle_s, BME280_REG_CTRL_MEAS, forced_cfg_s);
        do
        {
            if(once)
            {
                sl_sleeptimer_delay_millisecond(100);
                if((--sleep_iters) == 0)
                {
                    return SL_STATUS_IO_TIMEOUT;
                }
            }
            // Check if BME280 is still acquiring samples
            ret += bme280_read_register(i2cspm_handle_s, BME280_REG_STATUS, &data);
            once = true;
        } while(BME280_MEASURING_CHECK(data));
    }
    return ret;
}

/**
 * @brief Temperature compensation method leveraged from data sheet tweaked to return final float value
 * @see https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
 */
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
// t_fine carries fine temperature as global value
float bme280_get_temperature(void)
{
    sl_status_t ret = SL_STATUS_OK;
    uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
    int32_t var1, var2, adc_T, T;

    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_TEMP_MSB, 3, data);
    adc_T = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;

    var1 = (int32_t)((adc_T >> 3) - ((int32_t)calib_data_s.dig_T1 * 2));
    var1 = (var1 * ((int32_t)calib_data_s.dig_T2)) >> 11;
    var2 = (int32_t)((adc_T >> 4) - ((int32_t)calib_data_s.dig_T1));
    var2 = (((var2 * var2) >> 12) * ((int32_t)calib_data_s.dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return (float)T / 100;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
float bme280_get_pressure(void)
{
    sl_status_t ret = SL_STATUS_OK;
    uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
    int64_t var1, var2, var3, var4;

    bme280_get_temperature();

    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_PRESS_MSB, 3, data);
    int32_t adc_P = (data[0] << 16) | (data[1] << 8) | data[2];
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data_s.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data_s.dig_P5) * 131072);
    var2 = var2 + (((int64_t)calib_data_s.dig_P4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)calib_data_s.dig_P3) / 256) + ((var1 * ((int64_t)calib_data_s.dig_P2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)calib_data_s.dig_P1) >> 33;

    if(var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }

    var4 = 1048576 - adc_P;
    var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data_s.dig_P9) * (var4 >> 13) * (var4 >> 13)) >> 25;
    var2 = (((int64_t)calib_data_s.dig_P8) * var4) >> 19;
    var4 = ((var4 + var1 + var2) / 256) + (((int64_t)calib_data_s.dig_P7) * 16);

    return (float)var4 / 256.0;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of "47445" represents 47445/1024 = 46.333 %RH
float bme280_get_humidity(void)
{
    sl_status_t ret = SL_STATUS_OK;
    uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
    int32_t var1, var2, var3, var4, var5;

    bme280_get_temperature();

    ret += bme280_read_register_block(i2cspm_handle_s, BME280_REG_HUM_MSB, 2, data);
    int32_t adc_H = (data[0] << 8) | data[1];

    var1 = t_fine - ((int32_t)76800);
    var2 = (int32_t)(adc_H * 16384);
    var3 = (int32_t)(((int32_t)calib_data_s.dig_H4) * 1048576);
    var4 = ((int32_t)calib_data_s.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) >> 15;
    var2 = (var1 * ((int32_t)calib_data_s.dig_H6)) >> 10;
    var3 = (var1 * ((int32_t)calib_data_s.dig_H3)) >> 11;
    var4 = ((var2 * (var3 + (int32_t)32768)) >> 10) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calib_data_s.dig_H2)) + 8192) >> 14;
    var3 = var5 * var2;
    var4 = ((var3 >> 15) * (var3 >> 15)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calib_data_s.dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    uint32_t H = (uint32_t)(var5 >> 12);

    return (float)H / 1024.0;
}

float bme280_get_altitude(float seaLevel)
{
    float atmospheric = bme280_get_pressure() / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

float bme280_get_sealevel(float altitude, float atmospheric)
{
    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}
