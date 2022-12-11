/*
 * moisture_sensor.c
 *
 *  Created on: Oct 11, 2022
 *      Author: Caleb Provost
 */

#include "moisture_sensor.h"

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_vdac.h"
#include "em_prs.h"
#include "em_ldma.h"
#include "em_letimer.h"
#include "sl_pwm.h"
#include "sl_pwm_instances.h"
#include "sl_sleeptimer.h"
#include "app_log.h"

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

static bool moisture_sensor_initialized = false;
static bool moisture_sensor_calibrated = false;
static sl_pwm_instance_t pwm_instance = {0};
static ADC_TypeDef *adc_instance = 0;
static volatile uint32_t sample;
static volatile uint32_t millivolts;
moisture_cal_state_t ms_cal_state;
static float scaling_factor;
const float MAX_VAL = 100.0;

// For moisture cal
static volatile uint32_t millivolts_when_dry = 2200;
static volatile uint32_t millivolts_when_wet = 1100;

//sl_status_t guardener_sine_wave_init(moisture_sensor_cfg_t* cfg)
//{
//    sl_status_t ret = SL_STATUS_OK;
//
//    // Enable the PRS clock
//    CMU_ClockEnable(cmuClock_PRS, true);
//
//    // Set the level for PRS channel 0 and 1 to be high
//    PRS_LevelSet(PRS_SWLEVEL_CH0LEVEL | PRS_SWLEVEL_CH1LEVEL, _PRS_SWLEVEL_CH0LEVEL_MASK | _PRS_SWLEVEL_CH1LEVEL_MASK);
//
//    // Enable the VDAC clock
//    if(cfg->vdac == VDAC0) {
//        CMU_ClockEnable(cmuClock_VDAC0, true);
//    } else {
//        DLOGRET("Drivers not yet supporting other DAC peripherals");
//    }
//
//    // Initialize the VDAC
//    VDAC_Init(cfg->vdac, &cfg->dac_base_init);
//
//    // Initialize VDAC channel 0
//    VDAC_InitChannel(cfg->vdac, &cfg->dac0_init, 0);
//
//    // Initialize VDAC channel 1
//    VDAC_InitChannel(cfg->vdac, &cfg->dac1_init, 1);
//
//    // Set the settle time to zero for maximum update rate (mask it out)
//    VDAC0->OPA[0].TIMER &= ~(_VDAC_OPA_TIMER_SETTLETIME_MASK);
//
//    // Enable VDAC channels 0 and 1
//    VDAC_Enable(VDAC0, 0, true);
//    VDAC_Enable(VDAC0, 1, true);
//
//    return ret;
//}

uint32_t ms_get_millivolts()
{
    if(!moisture_sensor_initialized)
    {
        app_log_error("Moisture Sensor is not yet initialized");
        app_log_nl();
        return -1;
    }

    // Start the 500 kHz PWM signal
    sl_pwm_start(&pwm_instance);

    // Wait 100 ms for the RC moisture sensor circuit to attenuate
    sl_sleeptimer_delay_millisecond(100);

    // Start ADC conversion
    ADC_Start(adc_instance, adcStartSingle);

    // Wait for conversion to be complete
    while(!(adc_instance->STATUS & _ADC_STATUS_SINGLEDV_MASK));

    // Get ADC result
    sample = ADC_DataSingleGet(adc_instance);

    // Stop the 500 kHz PWM signal
    sl_pwm_stop(&pwm_instance);

    // Calculate input voltage in mV
    millivolts = (sample * 2500) / 4096;

    if(!moisture_sensor_calibrated)
    {
        app_log_warning("Moisture Sensor is not yet calibrated");
        app_log_nl();
    }

    return millivolts;
}

uint8_t ms_get_moisture_lvl(uint32_t millivolts)
{
    uint8_t level;
    if(moisture_sensor_calibrated)
    {
        if (millivolts > millivolts_when_wet)
        {
            level = (millivolts - millivolts_when_wet) * scaling_factor;
        }
        else
        {
            level = 0;
        }

        if(level > 100)
        {
            level = 100;
        }
    }
    else
    {
        app_log_warning("Moisture Sensor is not yet calibrated");
        app_log_nl();
    }

    return 100-level;
}

sl_status_t init_moisture_sensor(moisture_sensor_cfg_t* cfg)
{
    ret = SL_STATUS_OK;

    // PWM initialized by sl_pwm driver via TIMER1's CCO using PF7
    pwm_instance = cfg->pwm;
    sl_pwm_init_instances();
    sl_pwm_set_duty_cycle(&pwm_instance, 50);

    // Initialize the ADC if viable peripheral is given
    if(cfg->adc == ADC0)
    {
        adc_instance = cfg->adc;
        CMU_ClockEnable(cmuClock_ADC0, true);
    }
    else
    {
        ret = SL_STATUS_NOT_SUPPORTED;
        DLOGRET("Driver only has support for ADC0\n");
        return ret;
    }

    ADC_Init(adc_instance, &cfg->adc_base_cfg);
    ADC_InitSingle(adc_instance, &cfg->adc_single_cfg);

    moisture_sensor_initialized = true;

    return ret;
}

moisture_cal_state_t next_cal_state(moisture_cal_state_t cal_state)
{
    moisture_cal_state_t ret_state = cal_state;
    if(cal_state == CAL_DRY)
    {
        moisture_sensor_calibrated = false;
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("CAL_DRY:\n Moisture Sensor DRY: %lu mV  \n\t INSTRUCTIONS: Place finger(s) on senor, or place in a cup of water, then LONG PRESS Interactive Button to Continue", millivolts_when_dry);
        app_log_nl();
        #endif
        millivolts_when_dry = ms_get_millivolts();
        ret_state = CAL_WET;
    }
    else if(cal_state == CAL_WET)
    {
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("CAL_WET:\n Moisture Sensor WET: %lu mV, \n\t INSTRUCTIONS: Should go to CAL_OK", millivolts_when_wet);
        app_log_nl();
        #endif
        millivolts_when_wet = ms_get_millivolts();
        scaling_factor = normalize_moisture(millivolts_when_dry, millivolts_when_wet);
        ret_state = CAL_OK;
    }
    else if(cal_state == CAL_OK)
    {
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("Leaving CAL_OK\n"); // Moisture Sensor OK");
        app_log_nl();
        #endif
        ret_state = CAL_DRY;
    }
    else
    {
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("CAL Other state.... going back to DRY\n"); // Moisture Sensor OK");
        app_log_nl();
        #endif
        cal_state = CAL_DRY;
    }

    ms_cal_state = ret_state;
    return ret_state;
}

void moisture_get_cal_state(moisture_cal_state_t* _ext_cal_state)
{
  *_ext_cal_state = ms_cal_state;
}

float normalize_moisture(uint32_t dry, uint32_t wet)
{
    uint32_t scaling_diff;
    float ret_moisture = 0;

    if(wet > dry)
    {
        #if !APP_LOG_LEVEL_MASK_DEBUG
        app_log_debug("Moisture Sensor WET is more than DRY? Divider scaling_diff set to 1: %lu %lu mV", dry, wet);
        app_log_nl();
        #endif
        scaling_diff = 1;
    }
    else
    {
        scaling_diff = dry - wet;
    }

    ret_moisture = (dry * 1.0) / scaling_diff;

    #if !APP_LOG_LEVEL_MASK_DEBUG
    app_log_debug("Moisture Sensor DRY: %lu mV", dry);
    app_log_nl();
    #endif
    #if !APP_LOG_LEVEL_MASK_DEBUG
    app_log_debug("Moisture Sensor WET: %lu mV", wet);
    app_log_nl();
    #endif
    moisture_sensor_calibrated = true;

    return ret_moisture;
}
