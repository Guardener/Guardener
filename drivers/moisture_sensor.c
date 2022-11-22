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
#include "sl_sleeptimer.h"

#ifdef app_log_debug
#include "app_log.h"
#define DLOGRET(...)                                                                                                   \
    do {                                                                                                               \
        app_log_debug(__VA_ARGS__);                                                                                    \
        sl_status_print(ret);                                                                                          \
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
static sl_pwm_instance_t pwm_instance = {0};
static ADC_TypeDef *adc_instance = 0;
static volatile uint32_t sample;
static volatile uint32_t millivolts;

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
    if (!moisture_sensor_initialized)
    {
        DLOGRET("Moisture Sensor is not yet initialized");
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

    return millivolts;
}

sl_status_t init_moisture_sensor(moisture_sensor_cfg_t* cfg)
{
    sl_status_t ret = SL_STATUS_OK;

    // PWM initialized by sl_pwm driver via TIMER1's CCO using PF7
    pwm_instance = cfg->pwm;
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
