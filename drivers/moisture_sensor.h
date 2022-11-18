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
    /* DAC Specific Configurations */
    VDAC_TypeDef *vdac;                 /** VDAC Register Declaration */
    VDAC_Init_TypeDef dac_base_init;    /** VDAC initialization structure, common for both channels. */
    VDAC_InitChannel_TypeDef dac0_init; /** VDAC 0 channel initialization structure. */
    VDAC_InitChannel_TypeDef dac1_init; /** VDAC 1 channel initialization structure. */

    /* ADC Specific Configurations */
    CMU_Clock_TypeDef adc_clk_src;      /**< Peripheral clock to use for ADC */
    CMU_Osc_TypeDef adc_osc_type;       /**< Oscillator type */
    CMU_AUXHFRCOFreq_TypeDef tgt_freq;  /**< Target frequency of ADC */
    ADC_TypeDef *adc;                   /**< ADC instance */
    ADC_EM2ClockConfig_TypeDef em_mode; /**< Enable or disable EM2 ability */
    ADC_PosSel_TypeDef adc_channel;     /**< ADC channel */
    ADC_Ref_TypeDef ref_volts;          /**< Which reference voltage to use */
    ADC_AcqTime_TypeDef acq_time;       /**< Acquisition time (in ADC clock cycles) */
    ADC_PRSSEL_TypeDef prs_chan;        /**< PRS Channel to use */
    uint32_t captures_per_sample;       /**< Select single channel Data Valid level. SINGLE IRQ is set when (DVL+1) number of single channels have been converted and their results are available in the Single FIFO. */

    /* DMA Specific Configurations */
    CMU_Clock_TypeDef dma_clk_src;      /**< Peripheral clock to use DMA */
    uint8_t dma_channel;                /**< DMA channel */
    LDMA_PeripheralSignal_t dma_trig;   /**< What signal triggers the DMA to start */
    uint8_t *dest_buff;                 /**< Buffer were the ADC samples will be stored */
    uint32_t buff_size;                 /**< Size of the buffer */

    /* LETimer Specific Configurations */
    CMU_Osc_TypeDef let_osc_type;       /**< Oscillator type */
    CMU_Clock_TypeDef let_clk_src;      /**< Peripheral clock to use DMA */
    LETIMER_TypeDef *letimer;           /**< LETimer Peripheral to use */
    uint32_t delay_ms;                  /**< Time to wait till triggering ADC reading (ms) */
} moisture_sensor_cfg_t;
// @formatter:on

sl_status_t init_moisture_sensor(moisture_sensor_cfg_t* cfg);

#ifdef __cpluspluc
}
#endif
#endif /* DRIVERS_MOISTURE_SENSOR_H_ */
