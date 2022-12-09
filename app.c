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

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

#define SHORT_PRESS 2000
#define LONG_PRESS 20000

typedef struct {
    uint8_t lux_LO;
    uint8_t lux_MID;
    uint8_t lux_HI; // 0 to 16,777,216 lux
    uint8_t uvi_LO;
    uint8_t uvi_HI; // 0 to 65,535 uvi
    uint8_t ir_LO;
    uint8_t ir_HI; // 0 to 65,535 ir
    uint8_t temp_LO;
    uint8_t temp_HI; // -255.127 to 255.127 degrees celsius
    uint8_t humidity; // 0 to 255 percent relative humidity
    uint8_t mvolts_LO;
    uint8_t mvolts_HI; // 0 to 65,535 millivolts
} guardener_app_data_t;

// The advertisement data structure
#define APP_DATA_BYTES  (12)
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

// Helper union to make it easier to convert to BLE packets
typedef union {
    float f;
    struct
    {
        uint32_t m :23;
        uint32_t e :8;
        uint32_t s :1;
    } _f;
} guardener_float_t;

static guardener_adv_data_t guardener_adv_data = {0};

moisture_cal_state_t cal_state = CAL_START;
//extern volatile uint32_t millivolts_when_dry;
//extern volatile uint32_t millivolts_when_wet;

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
    app_log_info("Initializing Application Code \r\n");

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
        app_log_error("Failed to initialize the Si1145 sensor\r\n");
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
        app_log_error("Failed to initialize the BME280 sensor\r\n");
        while(true); // crash here
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

//    //* I2C TxRx needs to be above EM2 to Tx
//    SLEEP_SleepBlockBegin(sleepEM2);

//    //* Re-enable deeper sleeping states
//    SLEEP_SleepBlockEnd(sleepEM2);

    // Acquire data to be broadcast
    guardener_app_data_t app_data = {0};

    // Start Si1145 Test
    guardener_float_t lux, uvi, ir;
    if(si1145_get_lux_uvi_ir(&lux.f, &uvi.f, &ir.f, 15) != SL_STATUS_OK)
    {
        app_log_error("Failed to acquire lux, uvi, and ir readings \r\n");
        while(true); // crash here
    }
    else if(!calibrating)
    {
        app_log_info("lux=%0.2lf, uvi=%0.2lf, ir=%0.2lf", lux.f, uvi.f, ir.f);
    }

    /* Packet Construction Test */
    // lux received should be 0xAABBCC
    app_data.lux_LO  = 0xCC;
    app_data.lux_MID = 0xBB;
    app_data.lux_HI  = 0xAA;

    // uvi should be 0xDDEE
    app_data.uvi_LO = 0xEE;
    app_data.uvi_HI = 0xDD;

    // ir should be 0xFFAA
    app_data.ir_LO = 0xAA;
    app_data.ir_HI = 0xFF;

    // Start BME280 Test
    guardener_float_t temps, humid;
    if(sl_bme280_force_get_readings(&temps.f, NULL, &humid.f) != SL_STATUS_OK)
    {
        app_log_error("Failed to acquire temperature, pressure, and humidity readings \r\n");
        while(true); // crash here
    }
    else if(!calibrating)
    {
        app_log_info("temps=%0.2lf C, humid=%0.2lf %", temps.f, humid.f);
    }

    /**
     * To save space, parsing out two significant figures of the mantissa will allow us to send
     * more data overall. The packet structure will convert the 32 bits of a float to a 16 bit float.
     * 
     * float data:  sign [bit 31]; exponent [30-23]; mantissa [22-0]
     * packet data: sign [bit 15]; exponent [14-7];  mantissa [6-0]
     * 
     * This makes our min/max temp we can send over the air: -255.127 to 255.127
     */

    // temperature should receive -255.127
    guardener_float_t test;
    test.f = -255.127;

    uint16_t tmp_16 = 0;
    tmp_16 |= (((uint16_t)test._f.s & 0xFFFF) << 15);  // sign
    tmp_16 |= (((uint16_t)test._f.e & 0xFFFF) << 7);   // exponent
    tmp_16 |= (((uint16_t)test._f.m & 0xFFFF) << 0);   // mantissa

    app_data.temp_LO = tmp_16 & 0xFF;
    app_data.temp_HI = (tmp_16 >> 8) & 0xFF;

    // humidity should be 0x45
    app_data.humidity = 0x45;

    uint32_t mvolts = ms_get_millivolts();
    if(mvolts == (uint32_t)-1)
    {
        app_log_error("Failed to acquire moisture sensor value \r\n");
        while(true); // crash here
    }
    else if(!calibrating)
    {
        // if calibrated, can be % instead of mV
        app_log_info("mvolts=%lu mV", mvolts);
    }

    // millivols should be 0x0123
    app_data.mvolts_LO = 0x23;
    app_data.mvolts_HI = 0x01;

    if(interrupt_triggered)
    {
        interrupt_triggered = false; // debugger stop here to verify
        if(usr_btn_pressed == true)
        {
            calibrating = true; // Stops app log prints during calibration process
            app_log_info("User Interface Button has been pressed, Pressed time: %d released time:%d \r\n", pressed_time,
                         released_time);
        }
        else if(usr_btn_pressed == false)
        {
            app_log_info("User Interface Button has been released");
            if(released_time - pressed_time >= SHORT_PRESS && released_time - pressed_time < LONG_PRESS)
            {
                app_log_info(
                        "                                        ...after a SHORT PRESS, Interval of Pressed time: \t%d = (%d - %d) \r\n",
                        released_time - pressed_time, released_time, pressed_time);
            }
            else if(released_time - pressed_time >= LONG_PRESS)
            {
                // Let's have the procedure be this for now:
                // Long press 1, go from start to dry
                // Long press again to go to wet, set up and then press again, the
                // cal procedure will call the function to normalize

                cal_state = next_cal_state(cal_state);

                //else if(cfg->moisture_cal_state == CAL_NOT_OK)
                //  next_cal_state(&ms_cfg);

                app_log_info("calibration state is at %u", cal_state);
                app_log_info(
                        "                                        ...after a LONG PRESS, Interval of Pressed time: \t%d = (%d - %d) \r\n",
                        released_time - pressed_time, released_time, pressed_time);

                // Determine if calibration is done; return print statements if it is
                calibrating = (cal_state == CAL_OK) ? false : true;
            }

            //app_log_info("User Interface Button has been released, Pressed time: %d released time:%d \r\n", pressed_time, released_time);

            //%d.%d-b%d\n", evt->data.evt_system_boot.major,
            //        evt->data.evt_system_boot.minor, evt->data.evt_system_boot.patch,
            //        evt->data.evt_system_boot.build);
        }
    }

    /* Update the custom advertising packet's payload */
    strncpy((char *)guardener_adv_data.payload, (char *)&app_data, sizeof(app_data));
    if (sl_bt_legacy_advertiser_set_data(advertising_set_handle, 0, guardener_adv_data.data_size, (const uint8_t*)&guardener_adv_data) != SL_STATUS_OK)
    {
        app_log_error("Failed to set advertising data");
    }

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
            app_log_info("Bluetooth stack booted: v%d.%d.%d-b%d\n", evt->data.evt_system_boot.major,
                            evt->data.evt_system_boot.minor, evt->data.evt_system_boot.patch,
                            evt->data.evt_system_boot.build);

            // Extract unique ID from BT Address.
            sc = sl_bt_system_get_identity_address(&address, &address_type);
            app_assert_status(sc);

            app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                            address_type ? "static random" : "public device", address.addr[5], address.addr[4],
                            address.addr[3], address.addr[2], address.addr[1], address.addr[0]);

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

            // Set advertising interval to 20s.
            sc = sl_bt_advertiser_set_timing(advertising_set_handle, // advertising set handle
                    32000, // min. adv. interval (milliseconds * 1.6)
                    32000, // max. adv. interval (milliseconds * 1.6)
                    0,   // adv. duration
                    0);  // max. num. adv. events
            app_assert_status(sc);

            // Advertise on all channels
            sc = sl_bt_advertiser_set_channel_map(advertising_set_handle, 7 /*All channels*/);
            app_assert_status(sc);

            // Save some power by limiting the broadcast power
            sc = sl_bt_system_halt(1);
            app_assert_status(sc);
            sc = sl_bt_system_set_tx_power(-30, 0, NULL, NULL);
            app_assert_status(sc);
            sc = sl_bt_system_halt(0);
            app_assert_status(sc);

            /* Construct the advertisement packet with our desired initial data */
            // 1+2+12 bytes for type, company ID, and the payload
            guardener_adv_data.adv_len = 15;
            guardener_adv_data.adv_type = 0xFF; // Manufacture Custom
            guardener_adv_data.mfr_id_LO = 0xBABE & 0xFF;
            guardener_adv_data.mfr_id_HI = (0xBABE >> 8) & 0xFF;

            guardener_app_data_t app_data = {
                .lux_LO = 0x11,
                .lux_MID = 0x22,
                .lux_HI = 0x33,
                .uvi_LO = 0x44,
                .uvi_HI = 0x55,
                .ir_LO = 0x66,
                .ir_HI = 0x77,
                .temp_LO = 0x88,
                .temp_HI = 0x99,
                .humidity = 0xAA,
                .mvolts_LO = 0xBB,
                .mvolts_HI = 0xCC
            };
            strncpy((char *)guardener_adv_data.payload, (char *)&app_data, sizeof(app_data));

            // total length of advertising data
            guardener_adv_data.data_size = 1 + guardener_adv_data.adv_len;

            // Provide the BT stack the constructed advertisement packet
            sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle, 0, guardener_adv_data.data_size, (const uint8_t*)&guardener_adv_data);
            app_assert_status(sc);

            // Start advertising
            /**
            * sl_bt_advertiser_non_connectable           // (0x0) Non-connectable non-scannable.
            * sl_bt_advertiser_connectable_scannable     // (0x2) Undirected connectable scannable. This mode can only be used in legacy advertising PDUs.
            * sl_bt_advertiser_scannable_non_connectable // (0x3) Undirected scannable (Non-connectable but responds to scan requests).
            * sl_bt_advertiser_connectable_non_scannable // (0x4) Undirected connectable non-scannable. This mode can only be used in extended advertising PDUs.
            */
            sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
            app_assert_status(sc);

            app_log_info("Started advertising\n");
            break;

            // -------------------------------
            // This event indicates that a new connection was opened.
        case sl_bt_evt_connection_opened_id:
            app_log_info("Connection opened\n");

#ifdef SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT
            // Set remote connection power reporting - needed for Power Control
            sc = sl_bt_connection_set_remote_power_reporting(evt->data.evt_connection_opened.connection,
                                                             sl_bt_connection_power_reporting_enable);
            app_assert_status(sc);
#endif // SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT

            break;

        case sl_bt_evt_system_awake_id:
            // get all readings
            // prepare packet
            // send packet here or?
            app_log_info("Oh hi I'm awake now\n");
            break;

            // -------------------------------
            // This event indicates that a connection was closed.
        case sl_bt_evt_connection_closed_id:
            app_log_info("Connection closed\n");

            // Generate data for advertising
            sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle, sl_bt_advertiser_general_discoverable);
            app_assert_status(sc);

            // Restart advertising after client has disconnected.
            sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
            app_assert_status(sc);
            app_log_info("Started advertising\n");
            break;

        // Default event handler.
        default:
            app_log_info("event with no handler: 0x%X", event_id);
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
    app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                 address_type ? "static random" : "public device", address.addr[5], address.addr[4], address.addr[3],
                 address.addr[2], address.addr[1], address.addr[0]);
}
#endif // SL_CATALOG_CLI_PRESENT
