/**
 * si1145.c
 *
 *  Created on: Oct 10, 2022
 *      Author: Caleb Provost
 * 
 * @note Modeled off of SiLab's driver/si1133/src/sl_si1133.c
 */

#include <stdint.h>
#include <math.h>
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "si1145.h"
#include "app_log.h"

#define DLOGRET(...) do { app_log_debug("%s: ", __VA_ARGS__); sl_status_print(ret); } while(0)

#define SI1145_I2C_DEVICE_BUS_ADDRESS   (0x60)
#define SI1145_UV_COEFF0_VAL            (0x29)
#define SI1145_UV_COEFF1_VAL            (0x89)
#define SI1145_UV_COEFF2_VAL            (0x02)
#define SI1145_UV_COEFF3_VAL            (0x00)

// Needs to correspond with initialization of registers
#define SI1145_SELECTED_LUX_RANGE (1.0) // 1.0; 14.5
#define SI1145_SELECTED_LUX_GAIN  (0.3) // 0.3; 0.11; 0.06; 0.01; 0.008; 0.008
#define SI1145_SELECTED_UV_RANGE  (1.0) // 1.0; 14.5
#define SI1145_SELECTED_UV_GAIN   (0.3) // 0.3; 0.06; 0.03; 0.01
#define SI1145_REF_UV_OFFSET      (25.0)

static bool uv_high_range_s = false, lux_high_range_s = false;

sl_status_t
si1145_wait_until_sleep(sl_i2cspm_t *i2cspm);

/**************************************************************************//**
 *    Initializes the Si1145 chip
 *****************************************************************************/
sl_status_t si1145_init(sl_i2cspm_t *i2cspm) {
	app_log_debug("Temp Sensor Initialization\n");
	sl_status_t ret = SL_STATUS_OK;
	if (!i2cspm) {
		return SL_STATUS_NULL_POINTER;
	}

	/* Check that Si1145 is on the bus */
	uint8_t data = 0x00;
	ret += si1145_read_register(i2cspm, SI1145_REG_PART_ID, &data);
	if (data != SI1145_PART_ID_VAL) {
		return SL_STATUS_NOT_FOUND;
	}

	/* Reset the sensor. The reset function implements the necessary delays after reset. */
	ret += si1145_reset(i2cspm);
	DLOGRET("si1145_reset(i2cspm)");

	/* Enable UV Coefficients */
	ret += si1145_write_register(i2cspm, SI1145_REG_UCOEF0,
	SI1145_UV_COEFF0_VAL);
	ret += si1145_write_register(i2cspm, SI1145_REG_UCOEF1,
	SI1145_UV_COEFF1_VAL);
	ret += si1145_write_register(i2cspm, SI1145_REG_UCOEF2,
	SI1145_UV_COEFF2_VAL);
	ret += si1145_write_register(i2cspm, SI1145_REG_UCOEF3,
	SI1145_UV_COEFF3_VAL);
	DLOGRET("Enable UV Coefficients");

	/* Enable Sensor's Features */
	ret += si1145_set_parameter(i2cspm,
	SI1145_PARAM_CHLIST,
			SI1145_PARAM_CHLIST_EN_UV | SI1145_PARAM_CHLIST_EN_AUX
					| SI1145_PARAM_CHLIST_EN_EN_ALS_IR
					| SI1145_PARAM_CHLIST_EN_EN_ALS_VIS
					| SI1145_PARAM_CHLIST_EN_PS1);

	// Clear error counter & register
	ret += si1145_write_register(i2cspm, SI1145_REG_COMMAND, SI1145_CMD_NOP);

	// Put the module in ALS force mode
	ret += si1145_write_register(i2cspm, SI1145_REG_COMMAND,
	SI1145_CMD_ALS_FORCE);

	/* Monitor the temperature in the AUX ADC */
	ret += si1145_set_parameter(i2cspm, SI1145_PARAM_PS1_ADCMUX,
	SI1145_PARAM_PS1_ADCMUX_TEMP);
	DLOGRET("Enable Sensor's Features");

	/* Initialize the ADCs */
	ret += si1145_set_parameter(i2cspm, SI1145_PARAM_PS_ADC_GAIN,
			0 /* ADC Clock is divided by 1 */);
	ret += si1145_set_parameter(i2cspm, SI1145_PARAM_PS_ADC_COUNTER,
	SI1145_PARAM_PS_ADC_COUNTER_511_CLK);
	ret += si1145_set_parameter(i2cspm,
	SI1145_PARAM_PS_ADC_MISC, 0x20 /* High Signal Range (Gain divided by 14.5)*/
	| 0x04 /*ADC Clock is divided by 16*/);
	ret += si1145_set_parameter(i2cspm, SI1145_PARAM_ALS_IR_ADC_MISC,
			0x00 /*Normal Signal Range*/);
	ret += si1145_set_parameter(i2cspm, SI1145_PARAM_ALS_IR_ADC_GAIN,
			0 /*ADC Clock is divided by 1*/);
	ret += si1145_set_parameter(i2cspm, SI1145_PARAM_ALS_IR_ADC_COUNTER,
	SI1145_PARAM_ALS_IR_ADC_COUNTER_511_CLK);
	ret += si1145_set_parameter(i2cspm, SI1145_PARAM_ALS_IR_ADC_COUNTER,
			0x20 /* High Signal Range (Gain divided by 14.5)*/);
	lux_high_range_s = true;
	ret += si1145_set_parameter(i2cspm, SI1145_PARAM_ALS_VIS_ADC_GAIN,
			0 /*ADC Clock is divided by 1*/);
	ret += si1145_set_parameter(i2cspm, SI1145_PARAM_ALS_VIS_ADC_COUNTER,
	SI1145_PARAM_PS_ADC_COUNTER_511_CLK);
	ret += si1145_set_parameter(i2cspm, SI1145_PARAM_ALS_VIS_ADC_MISC,
			0x00 /*Normal Signal Range*/);
	DLOGRET("Initialized the ADCs");

	/* Enable Interrupts */
	ret += si1145_write_register(i2cspm, SI1145_REG_INT_CFG,
			0x00 /*INT pin is never driven*/);
	ret += si1145_write_register(i2cspm, SI1145_REG_INT_CFG,
	SI1145_REG_INT_CFG_PARAM_INT_OE);
	ret += si1145_write_register(i2cspm, SI1145_REG_IRQ_ENABLE,
	SI1145_REG_IRQ_ENABLE_PARAM_IRQ_ALS_IE);
	DLOGRET("Enable Interrupts");

#if INIT_PROX
  /* Initialize Proximity */
  ret += si1145_write_register(i2cspm, SI1145_REG_PS_LED21, 0x03); // 20mA for LED 1 only
  ret += si1145_set_parameter(i2cspm, SI1145_PARAM_PS1_ADCMUX,
                               SI1145_PARAM_PS1_ADCMUX_LARGE_IR);
  ret += si1145_set_parameter(i2cspm, SI1145_PARAM_PSLED12_SELECT,
                               SI1145_PARAM_PSLED12_SELECT_PS1_LED1);
  DLOGRET("Initialize Proximity");
#endif // INIT_PROX

	/* Set rate and run auto mode */
	// ret += si1145_write_register(i2cspm, SI1145_REG_MEAS_RATE0, 0xFF); // 255 * 31.25uS = 8ms
	ret += si1145_write_register(i2cspm, SI1145_REG_MEAS_RATE0, 0x00);
	ret += si1145_write_register(i2cspm, SI1145_REG_MEAS_RATE1, 0x00);
	ret += si1145_write_register(i2cspm, SI1145_REG_COMMAND,
	SI1145_CMD_PSALS_AUTO);
	DLOGRET("Set rate and run auto mode");

	if (ret != SL_STATUS_OK) {
		ret = SL_STATUS_INITIALIZATION;
	}

	return ret;
}

/***************************************************************************//**
 *    Stops the measurements on all channel and waits until the chip
 *    goes to sleep state.
 ******************************************************************************/
sl_status_t si1145_deinit(sl_i2cspm_t *i2cspm) {
	sl_status_t ret = SL_STATUS_OK;
	ret = si1145_set_parameter(i2cspm,
	SI1145_PARAM_CHLIST,
			SI1145_PARAM_CHLIST_EN_UV | SI1145_PARAM_CHLIST_EN_AUX
					| SI1145_PARAM_CHLIST_EN_EN_ALS_IR
					| SI1145_PARAM_CHLIST_EN_EN_ALS_VIS
					| SI1145_PARAM_CHLIST_EN_PS1);
	if (ret != SL_STATUS_OK) {
		return ret;
	}
	ret = si1145_pause_measurement(i2cspm);
	if (ret != SL_STATUS_OK) {
		return ret;
	}
	ret = si1145_wait_until_sleep(i2cspm);
	return ret;
}

/***************************************************************************//**
 *    Measure lux and UV index using the Si1145 sensor
 ******************************************************************************/
sl_status_t si1145_measure_lux_uvi(sl_i2cspm_t *i2cspm, float *lux, float *uvi) {
	sl_status_t ret = SL_STATUS_OK;
	float _lux;
	float _uvi;
	uint16_t vis, ir, uv;
	bool get_vis = false, get_ir = false, get_uv = false;
	uint8_t data1 = 0x00, data2 = 0x00;

	// Force a reading
	si1145_write_register(i2cspm, SI1145_REG_COMMAND, SI1145_CMD_ALS_FORCE);

	// Max wait ~ 9.84ms
	sl_sleeptimer_delay_millisecond(12);
	ret += si1145_read_register(i2cspm, SI1145_REG_RESPONSE, &data1);
	switch (data1) {
	case SI1145_ERRRSP_INVALID_SETTING:
	case SI1145_ERRRSP_PS1_ADC_OVERFLOW:
	case SI1145_ERRRSP_PS2_ADC_OVERFLOW:
	case SI1145_ERRRSP_PS3_ADC_OVERFLOW:
		app_log_error("Si1145 error: 0x%X\n", data1);
		get_vis = true;
		get_ir = true;
		get_uv = true;
		break;
	case SI1145_ERRRSP_ALS_VIS_ADC_OVERFLOW:
		app_log_error("Si1145 error: 0x%X\n", data1);
		vis = 0x7FFF;
		get_ir = true;
		get_uv = true;
		break;
	case SI1145_ERRRSP_ALS_IR_ADC_OVERFLOW:
		app_log_error("Si1145 error: 0x%X\n", data1);
		get_vis = true;
		ir = 0x7FFF;
		get_uv = true;
		break;
	case SI1145_ERRRSP_AUX_ADC_OVERFLOW:
		app_log_error("Si1145 error: 0x%X\n", data1);
		get_vis = true;
		get_ir = true;
		uv = SI1145_REF_UV_OFFSET;
		break;
	default:
		get_vis = true;
		get_ir = true;
		get_uv = true;
		break;
	}

	if (get_vis) {
		data1 = 0x00;
		data2 = 0x00;
		ret += si1145_read_register(i2cspm, SI1145_REG_ALS_VIS_DATA0, &data1);
		ret += si1145_read_register(i2cspm, SI1145_REG_ALS_VIS_DATA1, &data2);
		vis = data1 | (data2 << 8);
		DLOGRET("Retrieved VIS Data");
	}

	if (get_ir) {
		data1 = 0x00;
		data2 = 0x00;
		ret += si1145_read_register(i2cspm, SI1145_REG_ALS_IR_DATA0, &data1);
		ret += si1145_read_register(i2cspm, SI1145_REG_ALS_IR_DATA1, &data2);
		ir = data1 | (data2 << 8);
	}

	if (get_uv) {
		data1 = 0x00;
		data2 = 0x00;
		ret += si1145_read_register(i2cspm, SI1145_REG_AUX_DATA0_UVINDEX0,
				&data1);
		ret += si1145_read_register(i2cspm, SI1145_REG_AUX_DATA1_UVINDEX1,
				&data2);
		uv = data1 | (data2 << 8);
	}

	DLOGRET("Readings Acquired. Sending NOP");
	ret += si1145_write_register(i2cspm, SI1145_REG_COMMAND, SI1145_CMD_NOP);

	uint8_t irq_status;
	ret += si1145_read_register(i2cspm, SI1145_REG_IRQ_STATUS, &irq_status);
	DLOGRET("Checked IRQ Status: 0x%X", irq_status);
	ret += si1145_write_register(i2cspm, SI1145_REG_IRQ_STATUS, 0);

	// Temperature compensation: SI1145_REF_UV_OFFSET at 25C give about 35 ADC count per degree C
	if (lux_high_range_s) {
		_lux = vis;
	}
	_lux = vis - SI1145_SELECTED_LUX_GAIN * (uv - SI1145_REF_UV_OFFSET) / 35.0;

	if (uv_high_range_s) {
		_uvi = ir;
	}
	_uvi = ir - SI1145_SELECTED_UV_GAIN * (uv - SI1145_REF_UV_OFFSET) / 35.0;

	// compute the lux value - for an uncovered SI1145 die
	float _range_vis = ((!lux_high_range_s) ? 1.0 : 14.5);
	float _range_ir = ((!uv_high_range_s) ? 1.0 : 14.5);

	// See AN523.6 for conversion to lux equation
	_lux = (5.41f * _lux * _range_vis) / pow(2, SI1145_SELECTED_LUX_GAIN)
			+ (-0.08f * _uvi * _range_ir) / pow(2, SI1145_SELECTED_UV_GAIN);

	if (_lux < 0.0) {
		*lux = (float) 0.0;
	} else {
		*lux = _lux;
	}

	*uvi = _uvi;

	return ret;
}

/***************************************************************************//**
 *    Reads register from the Si1145 sensor
 ******************************************************************************/
sl_status_t si1145_read_register(sl_i2cspm_t *i2cspm, uint8_t reg,
		uint8_t *data) {
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef retval;
	uint8_t i2c_write_data[1];
	sl_status_t ret = ((sl_status_t) 0x0000);  ///< No error.

	seq.addr = SI1145_I2C_DEVICE_BUS_ADDRESS << 1;
	seq.flags = I2C_FLAG_WRITE_READ;
	/* Select register to start reading from */
	i2c_write_data[0] = reg;
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = 1;
	/* Select length of data to be read */
	seq.buf[1].data = data;
	seq.buf[1].len = 1;

	retval = I2CSPM_Transfer(i2cspm, &seq);
	if (retval != i2cTransferDone) {
		*data = 0xff;
		ret = SL_STATUS_TRANSMIT;
	}

	return ret;
}

/***************************************************************************//**
 *    Writes register in the Si1145 sensor
 ******************************************************************************/
sl_status_t si1145_write_register(sl_i2cspm_t *i2cspm, uint8_t reg,
		uint8_t data) {
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef retval;
	uint8_t i2c_write_data[2];
	uint8_t i2c_read_data[1];
	sl_status_t ret = SL_STATUS_OK;

	seq.addr = SI1145_I2C_DEVICE_BUS_ADDRESS << 1;
	seq.flags = I2C_FLAG_WRITE;
	/* Select register and data to write */
	i2c_write_data[0] = reg;
	i2c_write_data[1] = data;
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = 2;
	seq.buf[1].data = i2c_read_data;
	seq.buf[1].len = 0;

	retval = I2CSPM_Transfer(i2cspm, &seq);
	if (retval != i2cTransferDone) {
		ret = SL_STATUS_TRANSMIT;
	}

	return ret;
}

/***************************************************************************//**
 *    Writes a block of data to the Si1145 sensor.
 ******************************************************************************/
sl_status_t si1145_write_register_block(sl_i2cspm_t *i2cspm, uint8_t reg,
		uint8_t length, const uint8_t *data) {
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef retval;
	uint8_t i2c_write_data[10];
	uint8_t i2c_read_data[1];
	uint8_t i;
	sl_status_t ret = SL_STATUS_OK;

	seq.addr = SI1145_I2C_DEVICE_BUS_ADDRESS << 1;
	seq.flags = I2C_FLAG_WRITE;
	/* Select register to start writing to*/
	i2c_write_data[0] = reg;
	for (i = 0; i < length; i++) {
		i2c_write_data[i + 1] = data[i];
	}
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = length + 1;
	seq.buf[1].data = i2c_read_data;
	seq.buf[1].len = 0;

	retval = I2CSPM_Transfer(i2cspm, &seq);
	if (retval != i2cTransferDone) {
		ret = SL_STATUS_TRANSMIT;
	}

	return ret;
}

/***************************************************************************//**
 *    Reads a block of data from the Si1145 sensor.
 ******************************************************************************/
sl_status_t si1145_read_register_block(sl_i2cspm_t *i2cspm, uint8_t reg,
		uint8_t length, uint8_t *data) {
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef retval;
	uint8_t i2c_write_data[1];
	sl_status_t ret = SL_STATUS_OK;

	seq.addr = SI1145_I2C_DEVICE_BUS_ADDRESS << 1;
	seq.flags = I2C_FLAG_WRITE_READ;
	/* Select register to start reading from */
	i2c_write_data[0] = reg;
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = 1;
	/* Select length of data to be read */
	seq.buf[1].data = data;
	seq.buf[1].len = length;

	retval = I2CSPM_Transfer(i2cspm, &seq);
	if (retval != i2cTransferDone) {
		ret = SL_STATUS_TRANSMIT;
	}

	return ret;
}

/***************************************************************************//**
 * @brief
 *    Waits until the Si1145 is sleeping before proceeding
 *
 * @return
 *    @ret SL_STATUS_OK Success
 *    @ret SL_STATUS_TRANSMIT I2C transmit failure
 ******************************************************************************/
sl_status_t si1145_wait_until_sleep(sl_i2cspm_t *i2cspm) {
	uint8_t response;
	uint8_t count = 0;
	sl_status_t ret = SL_STATUS_OK;

	/* This loops until the Si1145 is known to be in its sleep state  */
	/* or if an i2c error occurs                                      */
	while (count < 5) {
		ret = si1145_read_register(i2cspm, SI1145_REG_RESPONSE, &response);
		if ((response & SI1145_RSP0_CHIPSTAT_MASK) == SI1145_RSP0_SLEEP) {
			break;
		}

		if (ret != SL_STATUS_OK) {
			return ret;
		}

		count++;
	}

	return ret;
}

/***************************************************************************//**
 *    Resets the Si1145
 ******************************************************************************/
sl_status_t si1145_reset(sl_i2cspm_t *i2cspm) {
	sl_status_t ret = SL_STATUS_OK;

	/* Perform the Reset Command */
	ret += si1145_write_register(i2cspm, SI1145_REG_MEAS_RATE0, 0);
	ret += si1145_write_register(i2cspm, SI1145_REG_MEAS_RATE1, 0);
	ret += si1145_write_register(i2cspm, SI1145_REG_IRQ_ENABLE, 0);
	ret += si1145_write_register(i2cspm, SI1145_REG_INT_CFG,
			0 /* SI1145_REG_INT_CFG_PARAM_INT_OE */);
	ret += si1145_write_register(i2cspm, SI1145_REG_IRQ_STATUS, 0xFF);
	DLOGRET("Registers cleared for reset");

	ret += si1145_write_register(i2cspm, SI1145_REG_COMMAND, SI1145_CMD_RESET);

	/* Allow a minimum of 25ms for Si1145 sensor to perform startup sequence */
	sl_sleeptimer_delay_millisecond(30);
	ret += si1145_write_register(i2cspm, SI1145_REG_HW_KEY, SI1145_HW_KEY_VAL);
	DLOGRET("Si1145 Reset");

	return ret;
}

/***************************************************************************//**
 *    Helper function to send a command to the Si1145
 ******************************************************************************/
sl_status_t si1145_send_command(sl_i2cspm_t *i2cspm, uint8_t command) {
	uint8_t response;
	uint8_t response_stored;
	uint8_t count = 0;
	uint32_t ret;

	/* Get the response register contents */
	ret = si1145_read_register(i2cspm, SI1145_REG_RESPONSE, &response_stored);
	if (ret != SL_STATUS_OK) {
		return ret;
	}

	response_stored = response_stored & SI1145_RSP0_CHIPSTAT_MASK;

	/* Double-check the response register is consistent */
	while (count < 5) {
		ret = si1145_wait_until_sleep(i2cspm);
		if (ret != SL_STATUS_OK) {
			return ret;
		}
		/* Skip if the command is RESET COMMAND COUNTER */
		if (command == SI1145_CMD_NOP) {
			break;
		}

		ret = si1145_read_register(i2cspm, SI1145_REG_RESPONSE, &response);

		if ((response & SI1145_RSP0_CHIPSTAT_MASK) == response_stored) {
			break;
		} else {
			if (ret != SL_STATUS_OK) {
				return ret;
			} else {
				response_stored = response & SI1145_RSP0_CHIPSTAT_MASK;
			}
		}

		count++;
	}

	/* Send the command */
	ret = si1145_write_register(i2cspm, SI1145_REG_COMMAND, command);
	if (ret != SL_STATUS_OK) {
		return ret;
	}

	count = 0;
	/* Expect a change in the response register */
	while (count < 5) {
		/* Skip if the command is RESET COMMAND COUNTER */
		if (command == SI1145_CMD_NOP) {
			break;
		}

		ret = si1145_read_register(i2cspm, SI1145_REG_RESPONSE, &response);
		if ((response & SI1145_RSP0_CHIPSTAT_MASK) != response_stored) {
			break;
		} else {
			if (ret != SL_STATUS_OK) {
				return ret;
			}
		}

		count++;
	}

	return SL_STATUS_OK;
}

/***************************************************************************//**
 *    Writes a byte to an Si1145 Parameter
 ******************************************************************************/
sl_status_t si1145_set_parameter(sl_i2cspm_t *i2cspm, uint8_t address,
		uint8_t value) {
	sl_status_t ret = SL_STATUS_OK;
	uint8_t buffer[2];
	uint8_t response_stored;
	uint8_t response;
	uint8_t count;

	ret = si1145_wait_until_sleep(i2cspm);
	if (ret != SL_STATUS_OK) {
		return ret;
	}

	si1145_read_register(i2cspm, SI1145_REG_RESPONSE, &response_stored);
	response_stored &= SI1145_RSP0_CHIPSTAT_MASK;

	buffer[0] = value;
	buffer[1] = 0x80 + (address & 0x3F);

	ret = si1145_write_register_block(i2cspm, SI1145_REG_PARAM_WR, 2,
			(uint8_t*) buffer);
	if (ret != SL_STATUS_OK) {
		return ret;
	}

	/* Wait for command to finish */
	count = 0;
	/* Expect a change in the response register */
	while (count < 5) {
		ret = si1145_read_register(i2cspm, SI1145_REG_RESPONSE, &response);
		if ((response & SI1145_RSP0_CHIPSTAT_MASK) != response_stored) {
			break;
		} else {
			if (ret != SL_STATUS_OK) {
				return ret;
			}
		}

		count++;
	}

	return SL_STATUS_OK;
}

/***************************************************************************//**
 *    Sends a PAUSE command to the Si1145
 ******************************************************************************/
sl_status_t si1145_pause_measurement(sl_i2cspm_t *i2cspm) {
	sl_status_t ret = si1145_send_command(i2cspm, SI1145_CMD_PS_PAUSE);
	ret += si1145_send_command(i2cspm, SI1145_CMD_ALS_PAUSE);
	ret += si1145_send_command(i2cspm, SI1145_CMD_PSALS_PAUSE);
	return ret;
}
