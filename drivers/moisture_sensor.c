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

static uint8_t dma_chan_s;
LDMA_TransferCfg_t trans;
LDMA_Descriptor_t descr;

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

/**************************************************************************//**
 * @brief LDMA Handler
 *****************************************************************************/
void LDMA_IRQHandler(void)
{
    // Clear interrupt flag
    LDMA_IntClear((1 << dma_chan_s) << _LDMA_IFC_DONE_SHIFT);
}

/**************************************************************************//**
 * @brief LETIMER initialization
 *****************************************************************************/
sl_status_t guardener_letimer_init(moisture_sensor_cfg_t* cfg)
{
    sl_status_t ret = SL_STATUS_OK;

    // Start LFRCO and wait until it is stable
    CMU_OscillatorEnable(cfg->let_osc_type, true, true);

    // Enable clock to the interface of the low energy modules
    CMU_ClockEnable(cmuClock_HFLE, true);

    // Route the LFRCO clock to LFA (TIMER0)
    if(cfg->let_osc_type == cmuOsc_LFRCO)
    {
        CMU_ClockSelectSet(cfg->let_clk_src, cmuSelect_LFRCO);
    }
    else
    {
        ret = SL_STATUS_NOT_SUPPORTED;
        DLOGRET("Driver only has support for LFRCO\n");
    }

    // Enable clock for LETIMER
    CMU_Clock_TypeDef timer;
    uint32_t prs_chan;
    PRS_Signal_t prs_sig;
    if(cfg->letimer == LETIMER0)
    {
        timer = cmuClock_LETIMER0;
        prs_chan = PRS_CH_CTRL_SOURCESEL_LETIMER0;
        prs_sig = (cfg->prs_chan == adcPRSSELCh1) ? PRS_CH_CTRL_SIGSEL_LETIMER0CH1 : PRS_CH_CTRL_SIGSEL_LETIMER0CH0;
    }
#if defined(CMU_LFACLKEN0_LETIMER1)
    else if (cfg->letimer == LETIMER1)
    {
        timer = cmuClock_LETIMER1;
        prs_chan = PRS_CH_CTRL_SOURCESEL_LETIMER1;
        prs_sig = (cfg->prs_chan == adcPRSSELCh1) ? PRS_CH_CTRL_SIGSEL_LETIMER1CH1 : PRS_CH_CTRL_SIGSEL_LETIMER1CH0;
    }
#endif
    else
    {
        ret = SL_STATUS_NOT_SUPPORTED;
        DLOGRET("Driver could not recognize which LETimer to initialize\n");
    }
    CMU_ClockEnable(timer, true);

    LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

    // Reload COMP0 on underflow, pulse output, and run once
    letimerInit.comp0Top = true;
    letimerInit.ufoa0 = letimerUFOAPulse;
    letimerInit.repMode = letimerRepeatFree; // letimerRepeatOneshot;

    // Initialize LETIMER
    LETIMER_Init(cfg->letimer, &letimerInit);

    // Need REP0 != 0 to run until the first event
    LETIMER_RepeatSet(cfg->letimer, 0, 1);

    // calculate the topValue
    uint32_t hertz = (1 / cfg->delay_ms) * 1000;
    uint32_t topValue = CMU_ClockFreqGet(timer) / hertz;

    // Compare on wake-up interval count
    LETIMER_CompareSet(cfg->letimer, 0, topValue);

    // Use timer as async PRS to trigger ADC
    CMU_ClockEnable(cmuClock_PRS, true);

    PRS_SourceAsyncSignalSet(cfg->prs_chan, prs_chan, prs_sig);

    return ret;
}

/**************************************************************************//**
 * @brief LDMA initialization
 *****************************************************************************/
sl_status_t guardener_ldma_init(moisture_sensor_cfg_t* cfg)
{
    sl_status_t ret = SL_STATUS_OK;

    // Enable CMU clock
    CMU_ClockEnable(cfg->dma_clk_src, true);

    // Basic LDMA configuration
    LDMA_Init_t ldmaInit = LDMA_INIT_DEFAULT;

    LDMA_Init(&ldmaInit);

    // Transfers trigger off ADC single conversion complete
    LDMA_TransferCfg_t t = LDMA_TRANSFER_CFG_PERIPHERAL(cfg->dma_trig);
    trans = t;

    // Provide the buffer and link relative offset (links to self)
    LDMA_Descriptor_t d = LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&(cfg->adc->SINGLEDATA), cfg->dest_buff, cfg->buff_size, 0);
    descr = d;

    // Transfer all the captured ADC samples; Quantity set by ADC_DVL
    descr.xfer.blockSize = cfg->captures_per_sample - 1;
    descr.xfer.ignoreSrec = true; // ignores single requests to save energy

    // Initialize transfer
    LDMA_StartTransfer(cfg->dma_channel, &trans, &descr);
    dma_chan_s = cfg->dma_channel;

    // Clear pending and enable interrupts for LDMA
    NVIC_ClearPendingIRQ(LDMA_IRQn);
    NVIC_EnableIRQ(LDMA_IRQn);

    return ret;
}

/**************************************************************************//**
 * @brief ADC initialization
 *****************************************************************************/
sl_status_t guardener_adc_init(moisture_sensor_cfg_t* cfg)
{
    sl_status_t ret = SL_STATUS_OK;

    // Declare init structs
    ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
    ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

    // Enable ADC clock
    if(cfg->adc == ADC0)
    {
        CMU_ClockEnable(cfg->adc_clk_src, true);
        CMU_ClockEnable(cmuClock_ADC0, true);
    }
    else
    {
        ret = SL_STATUS_NOT_SUPPORTED;
        DLOGRET("Driver only has support for ADC0\n");
    }

    // Set selected clock's frequency and use it to setup the ADC
    if(cfg->adc_osc_type == cmuOsc_AUXHFRCO)
    {
        if(cfg->em_mode != adcEm2Disabled)
        {
            // Select AUXHFRCO for ADC ASYNC mode so it can run in EM2
            CMU->ADCCTRL = CMU_ADCCTRL_ADC0CLKSEL_AUXHFRCO;
        }
        CMU_AUXHFRCOBandSet(cfg->tgt_freq);
        init.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
        init.prescale = ADC_PrescaleCalc(cfg->tgt_freq, CMU_AUXHFRCOBandGet());
    }
    else
    {
        ret = SL_STATUS_NOT_SUPPORTED;
        DLOGRET("Driver only has support for cmuOsc_AUXHFRCO\n");
    }

    // ADC Energy Mode Clock Configuration
    init.em2ClockConfig = cfg->em_mode;

    // Enable DMA Use
    initSingle.singleDmaEm2Wu = true;

    // Set the ADC channel
    initSingle.posSel = cfg->adc_channel;

    // Basic ADC single configuration
    initSingle.diff = false; // single-ended
    initSingle.reference = cfg->ref_volts; // user defined reference voltage
    initSingle.resolution = adcRes12Bit;  // 12-bit resolution
    initSingle.acqTime = cfg->acq_time;  // user defined acquisition time

    // Enable and set PRS channel
    initSingle.prsEnable = true;
    initSingle.prsSel = cfg->prs_chan;

    // Initialize ADC
    ADC_Init(cfg->adc, &init);
    ADC_InitSingle(cfg->adc, &initSingle);

    // Set single data valid level (DVL)
    cfg->adc->SINGLECTRLX = (cfg->adc->SINGLECTRLX & ~_ADC_SINGLECTRLX_DVL_MASK)
            | (((cfg->captures_per_sample - 1) << _ADC_SINGLECTRLX_DVL_SHIFT) & _ADC_SINGLECTRLX_DVL_MASK);

    // Clear the Single FIFO
    cfg->adc->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;

    return ret;
}

sl_status_t init_moisture_sensor(moisture_sensor_cfg_t* cfg)
{
    sl_status_t ret = SL_STATUS_OK;

    // Setup ADC to perform conversions via PRS
    ret += guardener_adc_init(cfg);
    DLOGRET("Setup ADC");

    // Setup DMA to move ADC results to memory
    ret += guardener_ldma_init(cfg);
    DLOGRET("Setup ADC's DMA");

    // Set up LETIMER to trigger ADC via PRS after set interval
    ret += guardener_letimer_init(cfg);
    DLOGRET("Setup ADC's LETIMER");

    return ret;
}
