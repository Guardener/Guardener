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

// @formatter:off
typedef struct {
    /* Output Signal Configurations */
    sl_pwm_instance_t pwm;
    /* ADC Specific Configurations */
    ADC_TypeDef *adc;
    ADC_Init_TypeDef adc_base_cfg;
    ADC_InitSingle_TypeDef adc_single_cfg;
} moisture_sensor_cfg_t;
// @formatter:on

sl_status_t init_moisture_sensor(moisture_sensor_cfg_t* cfg);

uint32_t ms_get_millivolts();

#ifdef __cpluspluc
}
#endif
#endif /* DRIVERS_MOISTURE_SENSOR_H_ */
