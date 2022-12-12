/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include <stdbool.h>
#include "em_common.h"
#include "sl_status.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_timer.h"
#include "app_log.h"
#include "em_rtcc.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#ifdef SL_CATALOG_CLI_PRESENT
#include "sl_cli.h"
#endif // SL_CATALOG_CLI_PRESENT
#include "app.h"
#include "sl_i2cspm_si1145_config.h"
#include "si1145.h"
#include "sl_i2cspm_bme280_config.h"
#include "sl_bme280.h"
#include "sl_pwm_instances.h"
#include "moisture_sensor.h"
#include "sl_button.h"

#define SHORT_PRESS 2000
#define LONG_PRESS 20000
#define DEBUG_IC_STATES 0
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#if DEBUG_IC_STATES
#include "sl_power_manager_config.h"
#include "gecko_sdk_4.1.3/platform/service/power_manager/src/sli_power_manager_private.h"
#endif // DEBUG_IC_STATES
#endif // SL_CATALOG_POWER_MANAGER_PRESENT

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// For printing errors
static char ret_buff[32] = {'\0'};

// floats are 32-bit
typedef struct {
    uint8_t lux_0;
    uint8_t lux_1;
    uint8_t lux_2;
    uint8_t lux_3;
    uint8_t uvi_0;
    uint8_t uvi_1;
    uint8_t uvi_2;
    uint8_t uvi_3;
    uint8_t ir_0;
    uint8_t ir_1;
    uint8_t ir_2;
    uint8_t ir_3;
    uint8_t temp_0;
    uint8_t temp_1;
    uint8_t temp_2;
    uint8_t temp_3;
    uint8_t hum_0;
    uint8_t hum_1;
    uint8_t hum_2;
    uint8_t hum_3;
    uint8_t moisture; // 0 to 255 percent saturation
    uint8_t ms_calib_state;
} guardener_app_data_t;

// The advertisement data structure
#define APP_DATA_BYTES  (22)
typedef struct {
    // length of the type + flags + payload
    uint8_t adv_len;
    // flag for type of packet; See Core Specification Supplement, Part A, Section 1.4
    uint8_t adv_type;
    // manufacture custom advertisement packets require manufacture ID
    uint8_t mfr_id_LO;
    uint8_t mfr_id_HI;
    // custom payload to be broadcasted
    uint8_t payload[APP_DATA_BYTES];

    // These values are NOT included in the actual advertising payload, just for bookkeeping
    char dummy;        // Space for null terminator
    uint8_t data_size; // Actual length of advertising data
} guardener_adv_data_t;

static guardener_adv_data_t guardener_adv_data = {0};

static inline void float_to_bytes(float * f_ptr, uint8_t * payload_data) {
    union {
        float f_val;
        uint8_t bytes[4];
     } encoder;

    encoder.f_val = *f_ptr;
    payload_data[0] = encoder.bytes[0];
    payload_data[1] = encoder.bytes[1];
    payload_data[2] = encoder.bytes[2];
    payload_data[3] = encoder.bytes[3];
}

static inline void update_adv_packet(guardener_adv_data_t *adv_data, float _l, float _u, float _i, float _t, float _h, uint8_t _m, uint8_t _c) {
    uint8_t _lux[4], _uvi[4], _ir[4], _tem[4], _hum[4];
    float_to_bytes(&_l, _lux);
    float_to_bytes(&_u, _uvi);
    float_to_bytes(&_i, _ir);
    float_to_bytes(&_t, _tem);
    float_to_bytes(&_h, _hum);
    int i = 0;
    for (int j=0; j<4; j++) {
        adv_data->payload[i++] = _lux[j];
    }
    for (int j=0; j<4; j++) {
        adv_data->payload[i++] = _uvi[j];
    }
    for (int j=0; j<4; j++) {
        adv_data->payload[i++] = _ir[j];
    }
    for (int j=0; j<4; j++) {
        adv_data->payload[i++] = _tem[j];
    }
    for (int j=0; j<4; j++) {
        adv_data->payload[i++] = _hum[j];
    }
    adv_data->payload[i++] = _m;
    adv_data->payload[i++] = _c;

    app_log_info("Client should receive 0x"
                    "%.2X%.2X%.2X%.2X" // lux
                    "%.2X%.2X%.2X%.2X" // uvi
                    "%.2X%.2X%.2X%.2X" // ir
                    "%.2X%.2X%.2X%.2X" // temp
                    "%.2X%.2X%.2X%.2X" // humid
                    "%.2X%.2X", // moisture && calibration state
                    adv_data->payload[0], adv_data->payload[1], adv_data->payload[2], adv_data->payload[3], // lux
                    adv_data->payload[4], adv_data->payload[5], adv_data->payload[6], adv_data->payload[7], // uvi
                    adv_data->payload[8], adv_data->payload[9], adv_data->payload[10], adv_data->payload[11], // ir
                    adv_data->payload[12], adv_data->payload[13], adv_data->payload[14], adv_data->payload[15], // temp
                    adv_data->payload[16], adv_data->payload[17], adv_data->payload[18], adv_data->payload[19], // humidity
                    adv_data->payload[20], adv_data->payload[21]); // moisture && calibration state
    app_log_nl();
    app_log_info("Translated to: lux=%.3f; uvi=%.3f; ir=%.3f; temp=%.3f C; humidity=%.3f%% RH; moisture=%d%% saturation; ms calib=%d",
                 _l, _u, _i, _t, _h, _m, _c);
    app_log_nl();
}

// Moisture calibration state flag
moisture_cal_state_t curr_cal_state = CAL_DRY;

// Button state.
static volatile bool usr_btn_pressed = false, interrupt_triggered = false, calibrating = false;

// Periodic timer handle.
static sl_simple_timer_t app_periodic_timer;

// BLE advertisement max interval is 10.24 seconds. this is 1/30th of what we want
//static volatile uint8_t ble_adv_interval_counter = 0;

uint32_t pressed_time;
uint32_t released_time;

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
    sl_status_t sc;
    app_log_info("Initializing Application Code");
    app_log_nl();

    // Init Si1145
    si1145_cfg_t init = { // @formatter:off
        .i2cspm = SL_I2CSPM_SI1145_PERIPHERAL, // I2C peripheral initialized already
        .uv = true,     // enable use of UV
        .temp = true,   // enable use of temperature
        .proxy = false, // disable use of proximity sensor
        .irq = false,   // disable interrupts
        .forced = true  // disable auto mode, use forced readings
    };    // @formatter:on
    sc = si1145_init(init);
    if(sc != SL_STATUS_OK)
    {
        app_log_error("Failed to initialize the Si1145 sensor");
        app_log_nl();
        while(true); // crash here
    }

    // Init BME280
    struct bme280_settings init_cfg = { // @formatter:off
        .osr_p = BME280_OVERSAMPLING_1X,            /*< pressure oversampling */
        .osr_t = BME280_OVERSAMPLING_1X,            /*< temperature oversampling */
        .osr_h = BME280_NO_OVERSAMPLING,            /*< humidity oversampling */
        .filter = BME280_FILTER_COEFF_OFF,          /*< filter coefficient */
        .standby_time = BME280_STANDBY_TIME_0_5_MS  /*< standby time */
    };    // @formatter:on
    sc = sl_bme280_init(SL_I2CSPM_BME280_PERIPHERAL, init_cfg, BME280_FORCED_MODE);
    if(sc != SL_STATUS_OK)
    {
        app_log_error("Failed to initialize the BME280 sensor");
        app_log_nl();
        while(true); // crash here
    }

    // Init process already sets BME280 to sleep, this is a sanity check
    if (sl_bme280_put_to_sleep() != SL_STATUS_OK)
    {
        app_log_error("Failed to put BME280 to sleep");
        app_log_nl();
    }

    // Initialize Moisture Sensor
    moisture_sensor_cfg_t ms_cfg = { // @formatter:off
        .pwm = sl_pwm_500k,
        .adc = ADC0,
        .adc_base_cfg = {
            .ovsRateSel = adcOvsRateSel2, /** Oversampling rate select. */
            .warmUpMode = adcWarmupNormal, /** ADC Warm-up mode. */
            .timebase = ADC_TimebaseCalc(0 /* 0 to use currently defined clock value */),
            .prescale = ADC_PrescaleCalc(cmuAUXHFRCOFreq_16M0Hz, 0),
            .tailgate = false, /** Enable/disable conversion tailgating. */
            .em2ClockConfig = adcEm2Disabled /** ADC EM2 clock configuration */
        },
        .adc_single_cfg = {
            .prsSel = adcPRSSELCh0, /** PRS trigger selection. Only if prsEnable is enabled. */
            .acqTime = adcAcqTime4, /** Acquisition time (in ADC clock cycles). */
            .reference = adcRefVDD, /** Sample reference selection. */
            .resolution = adcRes12Bit, /** Sample resolution. */
            .posSel = adcPosSelAPORT1XCH22, /** Positive input for single conversion mode. */
            .negSel = adcNegSelVSS, /** Negative input for single conversion mode. Grounded for non-differential conversion.  */
            .diff = false, /** Select if single-ended (false) or differential input (true). */
            .prsEnable = false, /** Peripheral reflex system trigger enable. */
            .leftAdjust = false, /** Select if left adjustment should be done. */
            .rep = false, /** Select if continuous conversion until explicit stop. */
            .singleDmaEm2Wu = false, /** When true, DMA is available in EM2 for single conversion */
            .fifoOverwrite = false /** When true, FIFO overwrites old data when full. If false, FIFO discards new data. */

        }
    };    // @formatter:on
    init_moisture_sensor(&ms_cfg);
}

static volatile bool once = true;
static inline void guardener_init_ble_advertiser()
{
    // Set advertising interval to 20s.
    sl_status_t sc = sc = sl_bt_advertiser_set_timing(advertising_set_handle, // advertising set handle
                                     32000, // min. adv. interval (milliseconds * 1.6)
                                     32000, // max. adv. interval (milliseconds * 1.6)
                                     0,   // adv. duration
                                     1);  // max. num. adv. events
    app_assert_status(sc);

    // Advertise on all channels
    sc = sl_bt_advertiser_set_channel_map(advertising_set_handle, 7 /*All channels*/);
    app_assert_status(sc);

    if (once)
    {
        // Save some power by limiting the broadcast power
        sc = sl_bt_system_halt(1);
        app_assert_status(sc);
        sc = sl_bt_system_set_tx_power(-30, 0, NULL, NULL);
        app_assert_status(sc);
        sc = sl_bt_system_halt(0);
        app_assert_status(sc);
        once = false;
    }

    /* Construct the advertisement packet with our desired initial data */
    // 1+2+19 bytes for type, company ID, and the payload
    guardener_adv_data.adv_len = 3+APP_DATA_BYTES;
    guardener_adv_data.adv_type = 0xFF; // Manufacture Custom
    guardener_adv_data.mfr_id_LO = 0xBA;
    guardener_adv_data.mfr_id_HI = 0xBE;

    guardener_app_data_t app_data = {
        .lux_0 = 0x11,
        .lux_1 = 0x22,
        .lux_2 = 0x33,
        .lux_3 = 0x44,
        .uvi_0 = 0x55,
        .uvi_1 = 0x66,
        .uvi_2 = 0x77,
        .uvi_3 = 0x88,
        .ir_0 = 0x99,
        .ir_1 = 0xAA,
        .ir_2 = 0xBB,
        .ir_3 = 0xCC,
        .temp_0 = 0xDD,
        .temp_1 = 0xEE,
        .temp_2 = 0xFF,
        .temp_3 = 0x00,
        .hum_0 = 0x11,
        .hum_1 = 0x22,
        .hum_2 = 0x33,
        .hum_3 = 0x44,
        .moisture = 0x55,
        .ms_calib_state = 0x66
    };
    strncpy((char *)guardener_adv_data.payload, (char *)&app_data, APP_DATA_BYTES);

    // total length of advertising data
    guardener_adv_data.data_size = 1 + guardener_adv_data.adv_len;

    // Provide the BT stack the constructed advertisement packet
    sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle, 0, guardener_adv_data.data_size, (const uint8_t*)&guardener_adv_data);
    app_assert_status(sc);
}


#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#if (SL_POWER_MANAGER_LOWEST_EM_ALLOWED != 1) && DEBUG_IC_STATES
// Table of energy modes counters. Each counter indicates the presence (not zero)
// or absence (zero) of requirements on a given energy mode. The table doesn't
// contain requirement on EM0.
static uint8_t requirement_em_table[SLI_POWER_MANAGER_EM_TABLE_SIZE] = {
  0,  // EM1 requirement counter
  0,  // EM2 requirement counter
};
/***************************************************************************//**
 * Get lowest energy mode to apply given the requirements on the different
 * energy modes.
 *
 * @return  Lowest energy mode: EM1, EM2 or EM3.
 *
 * @note If no requirement for any energy mode (EM1 and EM2), lowest energy mode
 * is EM3.
 ******************************************************************************/
static sl_power_manager_em_t get_lowest_em(void)
{
  uint32_t em_ix;
  sl_power_manager_em_t em;

  // Retrieve lowest Energy mode allowed given the requirements
  for (em_ix = 1; (em_ix < SL_POWER_MANAGER_LOWEST_EM_ALLOWED) && (requirement_em_table[em_ix - 1] == 0); em_ix++) {
    ;
  }

  em = (sl_power_manager_em_t)em_ix;

  return em;
}
#endif
static inline void update_and_enter_sleep_state(sl_power_manager_em_t em_new, bool add_new, sl_power_manager_em_t em_old, bool remove_old)
{
    if (remove_old)
    {
        // Remove the restriction
        sl_power_manager_remove_em_requirement(em_old);
    }

    if (add_new)
    {
        // Restrict the system from entering energy mode EM3 or lower
        sl_power_manager_add_em_requirement(em_new);
    }

    // Sleep to the lowest set EM state
    sl_power_manager_sleep();
#if DEBUG_IC_STATES // Debug only, extra time = extra power
    switch(get_lowest_em())
    {
        case SL_POWER_MANAGER_EM0:
            app_log_debug("MCU in EM0");
            break;
        case SL_POWER_MANAGER_EM1:
            app_log_debug("MCU in EM1");
            break;
        case SL_POWER_MANAGER_EM2:
            app_log_debug("MCU in EM2");
            break;
        case SL_POWER_MANAGER_EM3:
            app_log_debug("MCU in EM3");
            break;
        case SL_POWER_MANAGER_EM4:
            app_log_debug("MCU in EM4");
            break;
        default:
            break;
    }
    app_log_nl();
#endif // DEBUG_IC_STATES
}
#endif // defined(SL_CATALOG_POWER_MANAGER_PRESENT)

#ifndef SL_CATALOG_KERNEL_PRESENT
/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
    if(advertising_set_handle == 0xff)
    {
        // BT Stack not yet initialized, skip until it's ready
        return;
    }

    /**
     * I2C can enter EM3
     * BLE can enter EM1
     * PWM can enter EM2
     * ADC can enter EM3
     */
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Reset limits
    sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM3);
    sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM2);
    sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
    // preping the ble packets can be done in EM3
    update_and_enter_sleep_state(SL_POWER_MANAGER_EM3, true, 0/*ignored*/, false);
#endif

    // reinitialize the advertiser
    guardener_init_ble_advertiser();

    /**
     * Acquire all the data to be broadcast
     */
    sl_status_t ret = SL_STATUS_OK;
#if DEBUG_IC_STATES // Debug only, extra time = extra power
    // Verify the Si1145 is actually in sleep state
    uint8_t response;
    ret = si1145_read_register(SL_I2CSPM_SI1145_PERIPHERAL, SI1145_REG_RESPONSE, &response);
    if (ret == SL_STATUS_OK)
    {
        switch (response & SI1145_RSP0_CHIPSTAT_MASK)
        {
            case SI1145_RSP0_SLEEP:
                app_log_info("Si1145 is asleep");
                break;

            case SI1145_RSP0_SUSPEND:
                app_log_info("Si1145 is suspended");
                break;

            case SI1145_RSP0_RUNNING:
                app_log_info("Si1145 is actively running");
                break;
        }
        app_log_nl();
    }
    else
    {
        sl_status_to_string(ret, ret_buff, 32);
        app_log_error("Failed to I2C read if Si1145's current state: %s", ret_buff);
        app_log_nl();
    }
#endif // DEBUG_IC_STATES

    // I2C can be done in EM3, we do not need to remove or update the EM
    // Acquire Si1145's Readings
    float lux, uvi, ir;
    ret = si1145_get_lux_uvi_ir(&lux, &uvi, &ir, 15);
    if(ret != SL_STATUS_OK)
    {
        sl_status_to_string(ret, ret_buff, 32);
        app_log_error("Failed to acquire lux, uvi, and ir readings: %s", ret);
        app_log_nl();
    }
    else if(!calibrating)
    {
        app_log_info("lux=%0.2lf, uvi=%0.2lf, ir=%0.2lf", lux, uvi, ir);
        app_log_nl();
    }

#if DEBUG_IC_STATES
    // Si1145 should go to sleep after getting a reading, this is a sanity check to make sure it goes back to sleep
    ret = si1145_wait_until_sleep(SL_I2CSPM_SI1145_PERIPHERAL);
    if (ret != SL_STATUS_OK)
    {
        sl_status_to_string(ret, ret_buff, 32);
        app_log_error("Failed to put the Si1145 to sleep: %s", ret_buff);
        app_log_nl();
    }
#endif // DEBUG_IC_STATES

#if DEBUG_IC_STATES
    // Verify the BME280 is actually in sleep state
    if (sl_bme280_is_asleep())
    {
        app_log_info("BLE280 is asleep");
        app_log_nl();
    }
    else
    {
        app_log_info("BLE280 is awake");
        app_log_nl();
    }
#endif // DEBUG_IC_STATES

    // We should still be in EM3, we don't need to update EM yet
    // Acquire BME280's Readings
    float temps, humid;
    if(sl_bme280_force_get_readings(&temps, NULL, &humid) != SL_STATUS_OK)
    {
        app_log_error("Failed to acquire temperature, pressure, and humidity readings");
        app_log_nl();
    }
    else if(!calibrating)
    {
        app_log_info("temps=%0.2lf C, humid=%0.2lf %%", temps, humid);
        app_log_nl();
    }

#if DEBUG_IC_STATES
    ret = sl_bme280_put_to_sleep();
    if (ret != SL_STATUS_OK)
    {
        sl_status_to_string(ret, ret_buff, 32);
        app_log_error("Failed to put the BME280 to sleep: %s", ret_buff);
        app_log_nl();
    }
#endif // DEBUG_IC_STATES

    // Should still be in EM3, processing the data can still be done in EM3
//    float hum = sl_bme280_convert_bme2RH(humid); // returns NAN

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // PWM needs EM2 in order to work
    update_and_enter_sleep_state(SL_POWER_MANAGER_EM2, true, SL_POWER_MANAGER_EM3, true);
#endif

    // Acquire Moisture Sensor's millivolt Readings
    uint32_t mvolts = ms_get_millivolts();
    if(mvolts == (uint32_t)-1)
    {
        app_log_error("Failed to acquire moisture sensor value");
        app_log_nl();
    }
    else if((!calibrating) && (curr_cal_state != CAL_OK))
    {
        // if calibrated, can be % instead of mV
        app_log_info("moisture mvolts=%lu mV", mvolts);
        app_log_nl();
    }
    
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Higher state no longer needed, drop back to lower state
    update_and_enter_sleep_state(SL_POWER_MANAGER_EM3, true, SL_POWER_MANAGER_EM2, true);
#endif

    // Acquire Moisture Sensor's calibrated state
    moisture_get_cal_state(&curr_cal_state);
    uint8_t ms_calib = (uint8_t)curr_cal_state;

    // Acquire Moisture Sensor's Percent Saturation Readings
    uint8_t moisture_lvl = ms_get_moisture_lvl(mvolts);
    if(curr_cal_state == CAL_OK)
    {
        app_log_info("moisture saturation level= %lu %%", moisture_lvl);
        app_log_nl();
    }

    if(interrupt_triggered)
    {
        interrupt_triggered = false; // debugger stop here to verify
        if(usr_btn_pressed == true)
        {
            calibrating = true; // Stops app log prints during calibration process
            app_log_info("User Interface Button has been pressed, Pressed time: %d released time:%d",
                         pressed_time, released_time);
            app_log_nl();
        }
        else if(usr_btn_pressed == false)
        {
            app_log_info("User Interface Button has been released");
            if(released_time - pressed_time >= SHORT_PRESS && released_time - pressed_time < LONG_PRESS)
            {
                app_log_info("        ...after a SHORT PRESS, Interval of Pressed time: \t%d = (%d - %d)",
                             released_time - pressed_time, released_time, pressed_time);
                app_log_nl();
            }
            else if(released_time - pressed_time >= LONG_PRESS)
            {
                // Let's have the procedure be this for now:
                // Long press 1, go from start to dry
                // Long press again to go to wet, set up and then press again, the
                // cal procedure will call the function to normalize

                curr_cal_state = next_cal_state(curr_cal_state);

                app_log_info("calibration state is at %u", curr_cal_state);
                app_log_nl();
                app_log_info("        ...after a LONG PRESS, Interval of Pressed time: \t%d = (%d - %d) \r\n",
                             released_time - pressed_time, released_time, pressed_time);
                app_log_nl();

                // Determine if calibration is done; return print statements if it is
                calibrating = (curr_cal_state == CAL_OK) ? false : true;
            }
        }
    }

    /* Update the custom advertising packet's payload */
    update_adv_packet(&guardener_adv_data, lux, uvi, ir, temps, humid, moisture_lvl, ms_calib);
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // BLE needs at least EM2 in order to broadcast
    update_and_enter_sleep_state(SL_POWER_MANAGER_EM2, true, SL_POWER_MANAGER_EM3, true);
#endif
    if (sl_bt_legacy_advertiser_set_data(advertising_set_handle, 0, guardener_adv_data.data_size, (const uint8_t*)&guardener_adv_data) != SL_STATUS_OK)
    {
        app_log_error("Failed to set advertising data");
        app_log_nl();
    }

    // start the advertisement with the new packet
    app_assert_status(sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable));

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Enter deepest sleep now that application process is done (remove EM2 limit from above)
    update_and_enter_sleep_state(SL_POWER_MANAGER_EM3, true, SL_POWER_MANAGER_EM2, true);
#endif

    /**
     * Wait 5 minutes - time it takes to collect the data before allowing the
     * application loop to trigger sending another advertisement packet.
     */
//    sl_sleeptimer_delay_millisecond(5000); // test wait
    for (int i = 0; i<4; i++)
    {
        sl_sleeptimer_delay_millisecond(65535 /*max sleep interval*/);
    }
    sl_sleeptimer_delay_millisecond(37860 /*remaining amount of 300000 ms*/
                                    - 350 /* roughly the time it takes to acquire data*/);

    return;
}
#endif

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t* evt)
{
    sl_status_t sc;
    bd_addr address;
    uint8_t address_type;
    uint8_t system_id[8];
    uint32_t event_id = SL_BT_MSG_ID(evt->header);

    // Handle stack events
    switch(event_id)
    {
        // -------------------------------
        // This event indicates the device has started and the radio is ready.
        // Do not call any stack command before receiving this boot event!
        case sl_bt_evt_system_boot_id:
            // Print boot message.
            app_log_info("Bluetooth stack booted: v%d.%d.%d-b%d", evt->data.evt_system_boot.major,
                            evt->data.evt_system_boot.minor, evt->data.evt_system_boot.patch,
                            evt->data.evt_system_boot.build);
            app_log_nl();

            // Extract unique ID from BT Address.
            sc = sl_bt_system_get_identity_address(&address, &address_type);
            app_assert_status(sc);

            app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X",
                            address_type ? "static random" : "public device", address.addr[5], address.addr[4],
                            address.addr[3], address.addr[2], address.addr[1], address.addr[0]);
            app_log_nl();

            // Pad and reverse unique ID to get System ID.
            system_id[0] = address.addr[5];
            system_id[1] = address.addr[4];
            system_id[2] = address.addr[3];
            system_id[3] = 0xFF;
            system_id[4] = 0xFE;
            system_id[5] = address.addr[2];
            system_id[6] = address.addr[1];
            system_id[7] = address.addr[0];

            sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id, 0, sizeof(system_id), system_id);
            app_assert_status(sc);

            // Create an advertising set.
            sc = sl_bt_advertiser_create_set(&advertising_set_handle);
            app_assert_status(sc);

            // Initialize the advertiser
            guardener_init_ble_advertiser();

            // Start advertising
            /**
            * sl_bt_advertiser_non_connectable           // (0x0) Non-connectable non-scannable.
            * sl_bt_advertiser_connectable_scannable     // (0x2) Undirected connectable scannable. This mode can only be used in legacy advertising PDUs.
            * sl_bt_advertiser_scannable_non_connectable // (0x3) Undirected scannable (Non-connectable but responds to scan requests).
            * sl_bt_advertiser_connectable_non_scannable // (0x4) Undirected connectable non-scannable. This mode can only be used in extended advertising PDUs.
            */
            sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
            app_assert_status(sc);

            app_log_info("Started advertising");
            app_log_nl();
            break;

            // -------------------------------
            // This event indicates that a new connection was opened.
        case sl_bt_evt_connection_opened_id:
            app_log_info("Connection opened");
            app_log_nl();

#ifdef SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT
            // Set remote connection power reporting - needed for Power Control
            sc = sl_bt_connection_set_remote_power_reporting(evt->data.evt_connection_opened.connection,
                                                             sl_bt_connection_power_reporting_enable);
            app_assert_status(sc);
#endif // SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT

            break;

        // This event indicates that a connection was closed.
        case sl_bt_evt_connection_closed_id:
            app_log_info("Connection closed");
            app_log_nl();

            // Generate data for advertising
            sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle, sl_bt_advertiser_general_discoverable);
            app_assert_status(sc);

            // Restart advertising after client has disconnected.
            sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
            app_assert_status(sc);
            app_log_info("Started advertising");
            app_log_nl();
            break;

        // Default event handler.
        default:
            app_log_info("event with no handler: 0x%X", event_id);
            app_log_nl();
            break;
    }
}

/**************************************************************************//**
 * Callback function of connection close event.
 *
 * @param[in] reason Unused parameter required by the health_thermometer component
 * @param[in] connection Unused parameter required by the health_thermometer component
 *****************************************************************************/
void sl_bt_connection_closed_cb(uint16_t reason, uint8_t connection)
{
    (void)reason;
    (void)connection;
    sl_status_t sc;

    // Stop timer.
    sc = sl_simple_timer_stop(&app_periodic_timer);
    app_assert_status(sc);
}

/**************************************************************************//**
 * Simple Button
 * Button state changed callback
 * @param[in] handle Button event handle
 *****************************************************************************/
void sl_button_on_change(const sl_button_t* handle)
{
    interrupt_triggered = true; // for debugging

    // Button pressed.
    if(sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED && usr_btn_pressed == false)
    {
        if(&sl_button_usr_btn == handle)
        {
            usr_btn_pressed = true;
            pressed_time = RTCC_CounterGet();
            released_time = 0;
        }
    }
    // Button released. Requires a button to have been recorded-as-pressed
    else if(sl_button_get_state(handle) == SL_SIMPLE_BUTTON_RELEASED && usr_btn_pressed == true)
    {
        if(&sl_button_usr_btn == handle)
        {
            usr_btn_pressed = false;
            released_time = RTCC_CounterGet();
        }
    }
}

#ifdef SL_CATALOG_CLI_PRESENT
void hello(sl_cli_command_arg_t* arguments)
{
    (void)arguments;
    bd_addr address;
    uint8_t address_type;
    sl_status_t sc = sl_bt_system_get_identity_address(&address, &address_type);
    app_assert_status(sc);
    app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X",
                 address_type ? "static random" : "public device", address.addr[5], address.addr[4], address.addr[3],
                 address.addr[2], address.addr[1], address.addr[0]);
    app_log_nl();
}
#endif // SL_CATALOG_CLI_PRESENT
