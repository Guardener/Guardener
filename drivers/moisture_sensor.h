/*
 * moisture_sensor.h
 *
 *  Created on: Oct 11, 2022
 *      Author: Caleb Provost
 */

#ifndef DRIVERS_MOISTURE_SENSOR_H_
#define DRIVERS_MOISTURE_SENSOR_H_

#include <stdint.h>
#include "sl_status.h"
#include "sl_pwm.h"
#include "em_adc.h"
#include "em_vdac.h"
#include "em_cmu.h"
#include "em_ldma.h"

// Samples/interrupt
#define ADC_BUFFER_SIZE 8

#ifdef __cpluspluc
extern "C" {
#endif

typedef enum {
    CAL_DRY    = 0,
    CAL_WET    = 1,
    CAL_OK     = 2,
    CAL_NOT_OK = 3,
} moisture_cal_state_t;

// @formatter:off
typedef struct {
    /* Output Signal Configurations */
    sl_pwm_instance_t pwm;
    /* ADC Specific Configurations */
    ADC_TypeDef *adc;
    ADC_Init_TypeDef adc_base_cfg;
    ADC_InitSingle_TypeDef adc_single_cfg;

    /* Current state of cal */
    uint8_t cal_complete;
} moisture_sensor_cfg_t;
// @formatter:on

sl_status_t init_moisture_sensor(moisture_sensor_cfg_t* cfg);
uint8_t ms_get_moisture_lvl(uint32_t millivolts);
moisture_cal_state_t next_cal_state(moisture_cal_state_t cal);
void moisture_get_cal_state(moisture_cal_state_t* _ext_cal_state);
// take the wet and dry values and make that scale from 0 to 100
float normalize_moisture(uint32_t dry, uint32_t wet);
uint32_t ms_get_millivolts();

#ifdef __cpluspluc
}
#endif
#endif /* DRIVERS_MOISTURE_SENSOR_H_ */
