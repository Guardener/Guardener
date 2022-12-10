/*******************************************************************************
 * @file sl_bme280.h
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
#ifndef _DRIVERS_BME280_H_
#define _DRIVERS_BME280_H_

#include "app.h"
#include "bme280_defs.h"
#include "sl_status.h"
#include "sl_i2cspm.h"

#include <stdint.h>

#ifdef __cpluspluc
extern "C" {
#endif

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs.
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *  @return void.
 *
 */
void sl_bme280_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr       : Register address.
 *  @param[out] data          : Pointer to the data buffer to store the read data.
 *  @param[in] len            : No of bytes to read.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval 0 -> Success
 *  @retval > 0 -> Failure Info
 *
 */
int8_t sl_bme280_read_register(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr       : Register address.
 *  @param[in] data           : Pointer to the data buffer whose value is to be written.
 *  @param[in] len            : No of bytes to write.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 *  @return Status of execution
 *
 *  @retval BME280_OK -> Success
 *  @retval BME280_E_COMM_FAIL -> Communication failure.
 *
 */
int8_t sl_bme280_write_register(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);

/**
 * @brief
 *      Initializes the BME280
 * 
 * @param[in] i2cspm
 *      The initialized I2C peripheral which the BME280 is connected to
 * @param[in] cfg
 *      The configuration settings the BME280 should operate in
 * @param[in] mode
 *      The operation mode the BME280 should operate in
 * 
 * @return sl_status_t
 *      SiLab's status macros correlating with a failure type
*/
sl_status_t sl_bme280_init(sl_i2cspm_t* i2cspm, struct bme280_settings cfg, uint8_t mode);

/**
 * @brief
 *      Triggers a reading from the BME280 and updates any non-null parameters provided.
 *      If the BME280 is in sleep mode, it'll change to forced mode, acquire readings, then
 *      return to sleep mode.
 * 
 * @param[out] temp
 *      If not null, will be updated with °C for temperature
 * @param[out] pres
 *      If not null, will be updated with Pascal for pressure
 * @param[out] humi
 *      If not null, will be updated with % relative humidity
 * 
 * @return sl_status_t
 *      SiLab's status macros correlating with a failure type
*/
sl_status_t sl_bme280_force_get_readings(float *temp, float* pres, float* humi);

/**
 * @brief Puts the BME280 into sleep mode
*/
sl_status_t sl_bme280_put_to_sleep(void);

/**
 * @brief Wakes up the BME280 from sleep mode
*/
sl_status_t sl_bme280_wake_up_device(void);

/*!
 * @brief Convert BME280's float humidity reading to 0-100 % RH
 */
uint8_t sl_bme280_convert_bme2RH(float humi);

#ifdef __cpluspluc
}
#endif
#endif // _DRIVERS_BME280_H_
