/**
 * si1145.c
 *
 *  Created on: Oct 10, 2022
 *      Author: Caleb Provost
 *
 * @note Modeled off of SiLab's driver/si1133/src/sl_si1133.c
 */

#include "si1145.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "app_log.h"
#include <math.h>
#include <stdint.h>

static sl_status_t ret = SL_STATUS_OK;
#ifdef app_log_debug
#define DLOGRET(...)                                                                                                   \
    do {                                                                                                               \
        app_log_debug(__VA_ARGS__);                                                                                    \
        if (ret != SL_STATUS_OK) { \
            sl_status_print(ret);                                                                                          \
        } \
        app_log_nl();                                                                                                  \
    } while (0)
#else
#define DLOGRET(...) do { /* nop */ } while (0)
#define app_log_debug(...) do { /* nop */ } while (0)
#define app_log_info(...) do { /* nop */ } while (0)
#define app_log_error(...) do { /* nop */ } while (0)
#define app_log_nl(...) do { /* nop */ } while (0)
#endif

#define SI1145_I2C_DEVICE_BUS_ADDRESS (0x60)
#define SI1145_UV_COEFF0_VAL (0x29) // 0x7B
#define SI1145_UV_COEFF1_VAL (0x89) // 0x6B
#define SI1145_UV_COEFF2_VAL (0x02) // 0x01
#define SI1145_UV_COEFF3_VAL (0x00)

//static bool uv_high_range_s = false, lux_high_range_s = false;
static si1145_cfg_t cfg_s = {0};
sl_status_t si1145_wait_until_sleep(sl_i2cspm_t *i2cspm);

/**************************************************************************/ /**
 *    Initializes the Si1145 chip
 *****************************************************************************/
sl_status_t si1145_init(si1145_cfg_t cfg) {
    ret = SL_STATUS_OK;
    if (!cfg.i2cspm) {
        return SL_STATUS_NULL_POINTER;
    }
    cfg_s = cfg;

    /* Check that Si1145 is on the bus */
    uint8_t data = 0x00;
    ret += si1145_read_register(cfg_s.i2cspm, SI1145_REG_PART_ID, &data);
    if (data != SI1145_PART_ID_VAL) {
        return SL_STATUS_NOT_FOUND;
    }

    /* Reset the sensor. The reset function implements the necessary delays after reset. */
    ret += si1145_reset(cfg_s.i2cspm);
    DLOGRET("si1145_reset(cfg_s.i2cspm)");

    /* Enable UV Coefficients */
    ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_UCOEF0, SI1145_UV_COEFF0_VAL);
    ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_UCOEF1, SI1145_UV_COEFF1_VAL);
    ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_UCOEF2, SI1145_UV_COEFF2_VAL);
    ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_UCOEF3, SI1145_UV_COEFF3_VAL);
    DLOGRET("Enable UV Coefficients");

    uint8_t features = SI1145_PARAM_CHLIST_EN_UV | SI1145_PARAM_CHLIST_EN_EN_ALS_IR | SI1145_PARAM_CHLIST_EN_EN_ALS_VIS;
    if (cfg_s.temp) {
        features |= SI1145_PARAM_CHLIST_EN_AUX;
    }
    if (cfg_s.proxy) {
        features |= SI1145_PARAM_CHLIST_EN_PS1;
    }

    /* Enable Sensor's Features */
    ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_CHLIST, features);
    DLOGRET("Sensor's features En/disabled");

    /* Configure Interrupts */
    ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_INT_CFG, (cfg_s.irq) ? SI1145_REG_INT_CFG_PARAM_INT_OE : 0x00);
    ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_IRQ_ENABLE, SI1145_REG_IRQ_ENABLE_PARAM_IRQ_ALS_IE);
    DLOGRET("Interrupts En/disabled");

    if (cfg_s.temp) {
        /* Monitor the temperature in the AUX ADC */
      ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_PS1_ADCMUX, SI1145_PARAM_PS1_ADCMUX_TEMP);
      DLOGRET("Running Sensor's Temperature ADC");
    }

    // Clear error counter & register
    ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_COMMAND, SI1145_CMD_NOP);
    DLOGRET("Error registers cleared");

    if (cfg_s.proxy) {
        /* Initialize Proximity */
        ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_PS_LED21, 0x03); // 20mA for LED 1 only
        ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_PS1_ADCMUX, SI1145_PARAM_PS1_ADCMUX_LARGE_IR);
        ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_PSLED12_SELECT, SI1145_PARAM_PSLED12_SELECT_PS1_LED1);
        DLOGRET("Initialized Proximity Feature");

        ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_PS_ADC_GAIN, 0 /* ADC Clock is divided by 1 */);
        ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_PS_ADC_COUNTER, SI1145_PARAM_PS_ADC_COUNTER_511_CLK);
        /* High Signal Range (Gain divided by 14.5) | ADC Clock is divided by 16 */
        ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_PS_ADC_MISC, 0x20 | 0x04);
        DLOGRET("Initialized Proximity's ADCs");
    }

    /* Initialize IR */
    ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_ALS_IR_ADCMUX, SI1145_PARAM_PS1_ADCMUX_SMALL_IR);
    ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_ALS_IR_ADC_GAIN, 0 /*ADC Clock is divided by 1*/);
    ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_ALS_IR_ADC_COUNTER, SI1145_PARAM_ALS_IR_ADC_COUNTER_511_CLK);
    if (cfg_s.high_ir_range) {
        ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_ALS_IR_ADC_MISC, 0x20 /* High Signal Range (Gain divided by 14.5)*/);
    } else {
        ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_ALS_IR_ADC_MISC, 0x00 /*Normal Signal Range*/);
    }
    DLOGRET("Initialized IR Feature");

    /* Initialize Visibility */
    ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_ALS_VIS_ADC_GAIN, 0 /*ADC Clock is divided by 1*/);
    ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_ALS_VIS_ADC_COUNTER, SI1145_PARAM_PS_ADC_COUNTER_511_CLK);
    if (cfg_s.high_vis_range) {
        ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_ALS_VIS_ADC_MISC, 0x20 /* High Signal Range (Gain divided by 14.5)*/);
    } else {
        ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_ALS_VIS_ADC_MISC, 0x00 /*Normal Signal Range*/);
    }
    DLOGRET("Initialized Vis Feature");

    /* Set rate and mode */
    if (!cfg_s.forced) {
        ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_MEAS_RATE0, 0xFF); // 255 * 31.25uS = 8ms
    } else {
        ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_MEAS_RATE0, 0x00);
        ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_MEAS_RATE1, 0x00);
    }

    if (cfg_s.forced) {
        if (cfg_s.proxy) {
            ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_COMMAND, SI1145_CMD_ALS_FORCE);
            DLOGRET("Set rate and run forced mode");
        } else {
            ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_COMMAND, SI1145_CMD_PSALS_AUTO);
            DLOGRET("Set rate and run auto mode");
        }
    }

    if (ret != SL_STATUS_OK) {
        ret = SL_STATUS_INITIALIZATION;
    }

    return ret;
}

/*********************************************************************************/ /**
* Stops the measurements on all channel and waits until the chip goes to sleep state.
*************************************************************************************/
sl_status_t si1145_deinit() {
    ret = SL_STATUS_OK;
    uint8_t param = SI1145_PARAM_CHLIST_EN_UV | SI1145_PARAM_CHLIST_EN_EN_ALS_IR | SI1145_PARAM_CHLIST_EN_EN_ALS_VIS;
    if (cfg_s.temp) {
        param |= SI1145_PARAM_CHLIST_EN_AUX;
    }
    if (cfg_s.proxy) {
        param |= SI1145_PARAM_CHLIST_EN_PS1;
    }

    ret += si1145_set_parameter(cfg_s.i2cspm, SI1145_PARAM_CHLIST, param);

    if (ret != SL_STATUS_OK) {
        return ret;
    }

    ret = si1145_pause_measurement(cfg_s.i2cspm);
    if (ret != SL_STATUS_OK) {
        return ret;
    }

    ret = si1145_wait_until_sleep(cfg_s.i2cspm);
    return ret;
}

/***************************************************************************/ /**
 *    Measure LUX, UV index, and IR levels using the Si1145 sensor
 ******************************************************************************/
sl_status_t si1145_get_lux_uvi_ir(float *_lux, float *_uvi, float *_ir, int iter) {
    ret = SL_STATUS_OK;
    uint32_t vis = 0, ir = 0, uvi = 0, i = iter;
    uint8_t data[2] = {0};

    while (i) {
        // Force a reading
        ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_COMMAND, SI1145_CMD_ALS_FORCE);

        // Max wait ~ 9.84ms
        sl_sleeptimer_delay_millisecond(10);
        ret += si1145_read_register_block(cfg_s.i2cspm, SI1145_REG_RESPONSE, 2, data);

        // Error handling
        switch (data[0]) {
        case SI1145_ERRRSP_INVALID_SETTING:
            app_log_error("Si1145 error (0x%X): SI1145_ERRRSP_INVALID_SETTING\n", data[0]);
            break;
        case SI1145_ERRRSP_PS1_ADC_OVERFLOW:
            app_log_error("Si1145 error (0x%X): SI1145_ERRRSP_PS1_ADC_OVERFLOW\n", data[0]);
            break;
        case SI1145_ERRRSP_PS2_ADC_OVERFLOW:
            app_log_error("Si1145 error (0x%X): SI1145_ERRRSP_PS2_ADC_OVERFLOW\n", data[0]);
            break;
        case SI1145_ERRRSP_PS3_ADC_OVERFLOW:
            app_log_error("Si1145 error (0x%X): SI1145_ERRRSP_PS3_ADC_OVERFLOW\n", data[0]);
            break;
        case SI1145_ERRRSP_ALS_VIS_ADC_OVERFLOW:
            app_log_error("Si1145 error (0x%X): SI1145_ERRRSP_ALS_VIS_ADC_OVERFLOW\n", data[0]);
            break;
        case SI1145_ERRRSP_ALS_IR_ADC_OVERFLOW:
            app_log_error("Si1145 error (0x%X): SI1145_ERRRSP_ALS_IR_ADC_OVERFLOW\n", data[0]);
            break;
        case SI1145_ERRRSP_AUX_ADC_OVERFLOW:
            app_log_error("Si1145 error (0x%X): SI1145_ERRRSP_AUX_ADC_OVERFLOW\n", data[0]);
            break;
        default:
            if (ret != SL_STATUS_OK) {
                return ret;
            }
            break;
        }

        if (_lux != ((void*)0 /*null*/)) {
            ret += si1145_read_register(cfg_s.i2cspm, SI1145_REG_ALS_VIS_DATA0, &data[0]);
            ret += si1145_read_register(cfg_s.i2cspm, SI1145_REG_ALS_VIS_DATA1, &data[1]);
            vis += data[0] | (data[1] << 8);
        }

        if (_uvi != ((void*)0 /*null*/)) {
            ret += si1145_read_register(cfg_s.i2cspm, SI1145_REG_AUX_DATA0_UVINDEX0, &data[0]);
            ret += si1145_read_register(cfg_s.i2cspm, SI1145_REG_AUX_DATA1_UVINDEX1, &data[1]);
            uvi += data[0] | (data[1] << 8);
        }

        if (_ir != ((void*)0 /*null*/)) {
            ret += si1145_read_register(cfg_s.i2cspm, SI1145_REG_ALS_IR_DATA0, &data[0]);
            ret += si1145_read_register(cfg_s.i2cspm, SI1145_REG_ALS_IR_DATA1, &data[1]);
            ir += data[0] | (data[1] << 8);
        }

        i--;
    }

    /* Get the averaged values */
    if (_ir != ((void*)0 /*null*/)) {
        *_ir = (float) ir/iter;
    }

    if (_uvi != ((void*)0 /*null*/)) {
        *_uvi = (float) uvi/iter;
    }

    if (_lux != ((void*)0 /*null*/)) {
        float lux = (float) vis/iter;

        if (_ir != ((void*)0 /*null*/)) {
            /**
             * Equation from AN523 for LUX:
             *  [(ALS visible reading) - (ALS visible dark reading)] x (ALS visible coefficient)
             *      + [ (ALSIR reading) - (ALS IR dark reading)] x (ALS IR coefficient) ]
             *          x gain correction = LUX
             */
            // TODO: Acquire these values via calibration process from End User
            const uint16_t vis_dark_reading = 256;
            const uint16_t ir_dark_reading = 250;

            // CAUTION! Only checking if one of the ranges is high, both need to be the same!
            const float gain_correction = (cfg_s.high_vis_range) ? 14.5 : 1.0;

            // See Table 1 for Coefficients
            // TODO: Acquire these values via calibration process from End User
            const float vis_coeff = 5.41;
            const float ir_coeff = -0.08;

            lux = (vis - vis_dark_reading) * vis_coeff + ((ir - ir_dark_reading) * ir_coeff) * gain_correction;
        }

        *_lux = lux;
    }

    DLOGRET("Readings Acquired. Sending NOP"); // Resets any pending errors in the Si1145's registers
    ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_COMMAND, SI1145_CMD_NOP);

    if (cfg_s.irq) {
        uint8_t irq_status;
        ret += si1145_read_register(cfg_s.i2cspm, SI1145_REG_IRQ_STATUS, &irq_status);
        DLOGRET("Checked IRQ Status: 0x%X", irq_status);
        ret += si1145_write_register(cfg_s.i2cspm, SI1145_REG_IRQ_STATUS, 0);
    }

    return ret;
}

/***************************************************************************/ /**
 *    Reads register from the Si1145 sensor
 ******************************************************************************/
sl_status_t si1145_read_register(sl_i2cspm_t *i2cspm, uint8_t reg, uint8_t *data) {
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef retval;
    uint8_t i2c_write_data[1];
    ret = SL_STATUS_OK;

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

/***************************************************************************/ /**
 *    Writes register in the Si1145 sensor
 ******************************************************************************/
sl_status_t si1145_write_register(sl_i2cspm_t *i2cspm, uint8_t reg, uint8_t data) {
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef retval;
    uint8_t i2c_write_data[2];
    uint8_t i2c_read_data[1];
    ret = SL_STATUS_OK;

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

/***************************************************************************/ /**
 *    Writes a block of data to the Si1145
 *sensor.
 ******************************************************************************/
sl_status_t si1145_write_register_block(sl_i2cspm_t *i2cspm, uint8_t reg, uint8_t length, const uint8_t *data) {
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef retval;
    uint8_t i2c_write_data[10];
    uint8_t i2c_read_data[1];
    uint8_t i;
    ret = SL_STATUS_OK;

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

/***************************************************************************/ /**
 *    Reads a block of data from the
 *Si1145 sensor.
 ******************************************************************************/
sl_status_t si1145_read_register_block(sl_i2cspm_t *i2cspm, uint8_t reg, uint8_t length, uint8_t *data) {
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef retval;
    uint8_t i2c_write_data[1];
    ret = SL_STATUS_OK;

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

/***************************************************************************/ /**
 * @brief
 *    Waits until the Si1145 is sleeping
 *before proceeding
 *
 * @return
 *    @ret SL_STATUS_OK Success
 *    @ret SL_STATUS_TRANSMIT I2C transmit
 *failure
 ******************************************************************************/
sl_status_t si1145_wait_until_sleep(sl_i2cspm_t *i2cspm) {
    uint8_t response;
    uint8_t count = 0;
    ret = SL_STATUS_OK;

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

/***************************************************************************/ /**
 *    Resets the Si1145
 ******************************************************************************/
sl_status_t si1145_reset(sl_i2cspm_t *i2cspm) {
    ret = SL_STATUS_OK;

    /* Perform the Reset Command */
    ret += si1145_write_register(i2cspm, SI1145_REG_MEAS_RATE0, 0);
    ret += si1145_write_register(i2cspm, SI1145_REG_MEAS_RATE1, 0);
    ret += si1145_write_register(i2cspm, SI1145_REG_IRQ_ENABLE, 0);
    ret += si1145_write_register(i2cspm, SI1145_REG_INT_CFG, 0 /* SI1145_REG_INT_CFG_PARAM_INT_OE */);
    ret += si1145_write_register(i2cspm, SI1145_REG_IRQ_STATUS, 0xFF);
    DLOGRET("Registers cleared for reset");

    ret += si1145_write_register(i2cspm, SI1145_REG_COMMAND, SI1145_CMD_RESET);
    /* Allow a minimum of 25ms for Si1145 sensor to perform startup sequence */
    sl_sleeptimer_delay_millisecond(30);

    ret += si1145_write_register(i2cspm, SI1145_REG_HW_KEY, SI1145_HW_KEY_VAL);
    sl_sleeptimer_delay_millisecond(10);
    DLOGRET("Si1145 Reset");

    return ret;
}

/***************************************************************************/ /**
 *    Helper function to send a command to the Si1145
 ******************************************************************************/
sl_status_t si1145_send_command(sl_i2cspm_t *i2cspm, uint8_t command) {
    uint8_t response;
    uint8_t response_stored;
    uint8_t count = 0;
    ret = SL_STATUS_OK;

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

/***************************************************************************/ /**
 *    Writes a byte to an Si1145 Parameter
 ******************************************************************************/
sl_status_t si1145_set_parameter(sl_i2cspm_t *i2cspm, uint8_t address, uint8_t value) {
    ret = SL_STATUS_OK;
    uint8_t buffer[2];
    uint8_t response_stored;
    uint8_t response;
    uint8_t count;

//    ret = si1145_wait_until_sleep(i2cspm);
//    if (ret != SL_STATUS_OK) {
//        return ret;
//    }

    si1145_read_register(i2cspm, SI1145_REG_RESPONSE, &response_stored);
    response_stored &= SI1145_RSP0_CHIPSTAT_MASK;

    //buffer[0] = 0x80 + (address & 0x3F); // register location
    //buffer[1] = value;

    buffer[0] = value;
    buffer[1] = SI1145_CMD_PARAM_SET(address);
    ret = si1145_write_register_block(i2cspm, SI1145_REG_PARAM_WR, 2, buffer);
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

/***************************************************************************/ /**
 *    Sends a PAUSE command to the Si1145
 ******************************************************************************/
sl_status_t si1145_pause_measurement(sl_i2cspm_t *i2cspm) {
    ret = si1145_send_command(i2cspm, SI1145_CMD_PS_PAUSE);
    ret += si1145_send_command(i2cspm, SI1145_CMD_ALS_PAUSE);
    ret += si1145_send_command(i2cspm, SI1145_CMD_PSALS_PAUSE);
    return ret;
}
