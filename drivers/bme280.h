/*******************************************************************************
 * @file bme280.h
 *
 *  Created on: Oct 12, 2022
 *      Author: Caleb Provost
 *
 * @brief Bosch's BME280: Digital humidity, pressure and temperature sensor
 * @see   www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
 * @note  Code structure created following existing SiLabs driver methodology
 *
 ******************************************************************************/
#ifndef DRIVERS_BME280_H_
#define DRIVERS_BME280_H_

#include <stdint.h>
#include "sl_i2cspm.h"
#include "sl_status.h"

#ifdef __cpluspluc
extern "C" {
#endif

#define BME280_I2C_DEVICE_BUS_ADDRESS (0x76) // alt addr: 0x76 or 0x77

/**************************************************************************//**
 * @brief BME280 register definitions
 * @see Page 27 of Data Sheet
 ******************************************************************************/
#define BME280_REG_HUM_LSB    (0xFE) /**< Read Only Register; Reset State = 0x00 */
#define BME280_REG_HUM_MSB    (0xFD) /**< Read Only Register; Reset State = 0x80 */
#define BME280_REG_TEMP_XLSB  (0xFC) /**< Read Only Register; Reset State = 0x00 */
#define BME280_REG_TEMP_LSB   (0xFB) /**< Read Only Register; Reset State = 0x00 */
#define BME280_REG_TEMP_MSB   (0xFA) /**< Read Only Register; Reset State = 0x80 */
#define BME280_REG_PRESS_XLSB (0xF9) /**< Read Only Register; Reset State = 0x00 */
#define BME280_REG_PRESS_LSB  (0xF8) /**< Read Only Register; Reset State = 0x00 */
#define BME280_REG_PRESS_MSB  (0xF7) /**< Read Only Register; Reset State = 0x80 */
#define BME280_REG_CONFIG     (0xF5) /**< Read/Write Register; Reset State = 0x00 */
#define BME280_REG_CTRL_MEAS  (0xF4) /**< Read/Write Register; Reset State = 0x00 */
#define BME280_REG_STATUS     (0xF3) /**< Read Only Register; Reset State = 0x00 */
#define BME280_STATUS_UPDATING_BIT (0)
#define BME280_STATUS_MEASURING_BIT (3)
#define BME280_REG_CTRL_HUM   (0xF2) /**< Read Only Register; Reset State = 0x00 */
#define BME280_REG_CALIB26    (0xE1) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB27    (0xE2) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB28    (0xE3) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB29    (0xE4) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB30    (0xE5) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB31    (0xE6) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB32    (0xE7) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB33    (0xE8) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB34    (0xE9) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB35    (0xEA) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB36    (0xEB) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB37    (0xEC) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB38    (0xED) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB39    (0xEE) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB40    (0xEF) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB41    (0xF0) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_RESET      (0xE0) /**< Write Only Register; Reset State = 0x00 */
#define BME280_REG_ID         (0xD0) /**< Read Only Register; Reset State = 0x60 */
#define BME280_REG_CALIB00    (0x88) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB01    (0x89) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB02    (0x8A) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB03    (0x8B) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB04    (0x8C) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB05    (0x8D) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB06    (0x8E) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB07    (0x8F) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB08    (0x90) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB09    (0x91) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB10    (0x92) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB11    (0x93) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB12    (0x94) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB13    (0x95) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB14    (0x96) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB15    (0x97) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB16    (0x98) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB17    (0x99) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB18    (0x9A) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB19    (0x9B) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB20    (0x9C) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB21    (0x9D) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB22    (0x9E) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB23    (0x9F) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB24    (0xA0) /**< Read Only Calibration Register; Reset State = individual */
#define BME280_REG_CALIB25    (0xA1) /**< Read Only Calibration Register; Reset State = individual */

#define BME280_CHIP_ID        (0x60) /**< The expected response fron BME280_REG_ID */
#define BME280_UPDATING_CHECK(status_ret) (status_ret & (1 << BME280_STATUS_UPDATING_BIT))
#define BME280_MEASURING_CHECK(resp)  (resp & (1 << BME280_STATUS_MEASURING_BIT))

/**************************************************************************//**
 * @brief BME280 Register Commands
 ******************************************************************************/
#define BME280_CMD_RESET_POWER_ON                 (0xB6)  /**< When written to BME280_REG_RESET, device is reset using the complete power-on-reset procedure. */
#define BEM280_CMD_CTRL_MEAS_OSRS_T_0X_SAMPLING   (0x00)  /**< Data acquisition sampling rates for Temperature */
#define BEM280_CMD_CTRL_MEAS_OSRS_T_1X_SAMPLING   (0x20)  /**< Data acquisition sampling rates for Temperature */
#define BEM280_CMD_CTRL_MEAS_OSRS_T_2X_SAMPLING   (0x40)  /**< Data acquisition sampling rates for Temperature */
#define BEM280_CMD_CTRL_MEAS_OSRS_T_4X_SAMPLING   (0x60)  /**< Data acquisition sampling rates for Temperature */
#define BEM280_CMD_CTRL_MEAS_OSRS_T_8X_SAMPLING   (0x80)  /**< Data acquisition sampling rates for Temperature */
#define BEM280_CMD_CTRL_MEAS_OSRS_T_16X_SAMPLING  (0x50)  /**< Data acquisition sampling rates for Temperature */
#define BEM280_CMD_CTRL_MEAS_OSRS_P_0X_SAMPLING   (0x00)  /**< Data acquisition sampling rates for Pressure */
#define BEM280_CMD_CTRL_MEAS_OSRS_P_1X_SAMPLING   (0x04)  /**< Data acquisition sampling rates for Pressure */
#define BEM280_CMD_CTRL_MEAS_OSRS_P_2X_SAMPLING   (0x08)  /**< Data acquisition sampling rates for Pressure */
#define BEM280_CMD_CTRL_MEAS_OSRS_P_4X_SAMPLING   (0x0C)  /**< Data acquisition sampling rates for Pressure */
#define BEM280_CMD_CTRL_MEAS_OSRS_P_8X_SAMPLING   (0x10)  /**< Data acquisition sampling rates for Pressure */
#define BEM280_CMD_CTRL_MEAS_OSRS_P_16X_SAMPLING  (0x14)  /**< Data acquisition sampling rates for Pressure */
#define BEM280_CMD_CTRL_MEAS_MODE_NORMAL          (0x03)  /**< Controls the sensor mode of the device */
#define BEM280_CMD_CTRL_MEAS_MODE_FORCE           (0x02)  /**< Controls the sensor mode of the device */
#define BEM280_CMD_CTRL_MEAS_MODE_FORCED          (0x01)  /**< Controls the sensor mode of the device */
#define BEM280_CMD_CTRL_MEAS_MODE_SLEEP           (0x00)  /**< Controls the sensor mode of the device */
#define BME280_REG_CTRL_HUM_OSRS_H_0X_SAMPLING    (0x00)  /**< Data acquisition sampling rates for Humidity */
#define BME280_REG_CTRL_HUM_OSRS_H_1X_SAMPLING    (0x01)  /**< Data acquisition sampling rates for Humidity */
#define BME280_REG_CTRL_HUM_OSRS_H_2X_SAMPLING    (0x02)  /**< Data acquisition sampling rates for Humidity */
#define BME280_REG_CTRL_HUM_OSRS_H_4X_SAMPLING    (0x03)  /**< Data acquisition sampling rates for Humidity */
#define BME280_REG_CTRL_HUM_OSRS_H_8X_SAMPLING    (0x04)  /**< Data acquisition sampling rates for Humidity */
#define BME280_REG_CTRL_HUM_OSRS_H_16X_SAMPLING   (0x05)  /**< Data acquisition sampling rates for Humidity */
#define BME280_REG_CONFIG_T_SB_0_5_MS             (0x00)  /**< 0.5 ms inactive standby duration in normal mode */
#define BME280_REG_CONFIG_T_SB_62_5_MS            (0x20)  /**< 62.5 ms inactive standby duration in normal mode */
#define BME280_REG_CONFIG_T_SB_125_MS             (0x40)  /**< 125 ms inactive standby duration in normal mode */
#define BME280_REG_CONFIG_T_SB_250_MS             (0x60)  /**< 250 ms inactive standby duration in normal mode */
#define BME280_REG_CONFIG_T_SB_500_MS             (0x80)  /**< 500 ms inactive standby duration in normal mode */
#define BME280_REG_CONFIG_T_SB_1_S                (0xA0)  /**< 1000 ms inactive standby duration in normal mode */
#define BME280_REG_CONFIG_T_SB_10_MS              (0xC0)  /**< 10 ms inactive standby duration in normal mode */
#define BME280_REG_CONFIG_T_SB_20_MS              (0xE0)  /**< 20 ms inactive standby duration in normal mode */
#define BME280_REG_CONFIG_I2C                     (0x00)  /**< Enables I2C Bus Interface */
#define BME280_REG_CONFIG_SPI                     (0x01)  /**< Enables SPI Bus Interface */
#define BME280_REG_CONFIG_FILTER_OFF              (0x00)  /**< 1 Step Filtering Response */
#define BME280_REG_CONFIG_FILTER_2                (0x02)  /**< 2 Step Filtering Response */
#define BME280_REG_CONFIG_FILTER_4                (0x04)  /**< 5 Step Filtering Response */
#define BME280_REG_CONFIG_FILTER_8                (0x06)  /**< 11 Step Filtering Response */
#define BME280_REG_CONFIG_FILTER_16               (0x08)  /**< 22 Step Filtering Response */

/**************************************************************************//**
 * @brief BME280 Temp, Pressure, & Humidity Compensation Parameters
 * @see Page 24 of Data Sheet
 * @note From Data Sheet:
 *            The trimming parameters are programmed into the devices’ 
 *            non-volatile memory (NVM) during production and cannot be altered
 *            by the customer. Each compensation word is a 16-bit signed or 
 *            unsigned integer value stored in two’s complement. As the memory
 *            is organized into 8-bit words, two words must always be combined
 *            in order to represent the compensation word. The 8-bit registers
 *            are named calib00…calib41 and are stored at memory addresses
 *            0x88…0xA1 and 0xE1…0xE7. The corresponding compensation words
 *            are named dig_T# for temperature compensation related values,
 *            dig_P# for pressure related values and dig_H# for humidity
 *            related values.
 ******************************************************************************/
typedef struct
{
    uint16_t dig_T1; /**< Corresponding Register Location: 0x88 / 0x89 */
    int16_t dig_T2; /**< Corresponding Register Location: 0x8A / 0x8B */
    int16_t dig_T3; /**< Corresponding Register Location: 0x8C / 0x8D */
    uint16_t dig_P1; /**< Corresponding Register Location: 0x8E / 0x8F */
    int16_t dig_P2; /**< Corresponding Register Location: 0x90 / 0x91 */
    int16_t dig_P3; /**< Corresponding Register Location: 0x92 / 0x93 */
    int16_t dig_P4; /**< Corresponding Register Location: 0x94 / 0x95 */
    int16_t dig_P5; /**< Corresponding Register Location: 0x96 / 0x97 */
    int16_t dig_P6; /**< Corresponding Register Location: 0x98 / 0x99 */
    int16_t dig_P7; /**< Corresponding Register Location: 0x9A / 0x9B */
    int16_t dig_P8; /**< Corresponding Register Location: 0x9C / 0x9D */
    int16_t dig_P9; /**< Corresponding Register Location: 0x9E / 0x9F */
    uint8_t dig_H1; /**< Corresponding Register Location: 0xA1  */
    int16_t dig_H2; /**< Corresponding Register Location: 0xE1 / 0xE2 */
    uint8_t dig_H3; /**< Corresponding Register Location: 0xE3  */
    int16_t dig_H4; /**< Corresponding Register Location: 0xE4 / 0xE5[3:0]  */
    int16_t dig_H5; /**< Corresponding Register Location: 0xE5[7:4] / 0xE6  */
    int8_t dig_H6; /**< Corresponding Register Location: 0xE7  */
} bme280_calib_t;

typedef struct
{
    uint8_t opt_mode; /**< Operational mode of micro */
    uint8_t acqu_intvl; /**< Intervals between sampling; Ignored in FORCED mode */
    uint8_t temp_samp_rate; /**< Sampling rate for temperature */
    uint8_t hum_samp_rate; /**< Sampling rate for humidity */
    uint8_t pres_samp_rate; /**< Sampling rate for pressure */
    uint8_t filter; /**< IIR Filtering for acquired samples */
} bme280_init_t;

sl_status_t bme280_init(sl_i2cspm_t* i2cspm, bme280_init_t init);
sl_status_t bme280_deinit(void);
sl_status_t bme280_force_trigger_measurements(void);
float bme280_get_temperature(void);
float bme280_get_pressure(void);
float bme280_get_humidity(void);
float bme280_get_altitude(float seaLevel);
float bme280_get_sealevel(float altitude, float atmospheric);

#ifdef __cpluspluc
}
#endif
#endif // DRIVERS_bme280_H_
