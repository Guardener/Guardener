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
    uint8_t humidity; // 0 to 255 percent relative humidity
    uint8_t moisture; // 0 to 255 percent saturation
} guardener_app_data_t;

// The advertisement data structure
#define APP_DATA_BYTES  (18)
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

static inline void update_adv_packet(guardener_adv_data_t *adv_data, float _l, float _u, float _i, float _t, uint8_t _h, uint8_t _m) {
    uint8_t _lux[4], _uvi[4], _ir[4], _tem[4];
    float_to_bytes(&_l, _lux);
    float_to_bytes(&_u, _uvi);
    float_to_bytes(&_i, _ir);
    float_to_bytes(&_t, _tem);
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
    adv_data->payload[i++] = _h;
    adv_data->payload[i++] = _m;
    
    app_log_warning("Client should receive 0x%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X; Translated to:\n\r"
                    "lux=%.3f; uvi=%.3f; ir=%.3f; temp=%.3f C; humidity=%d%% RH; moisture=%d%% saturation",
                    adv_data->payload[0], adv_data->payload[1], adv_data->payload[2], adv_data->payload[3], // lux
                    adv_data->payload[4], adv_data->payload[5], adv_data->payload[6], adv_data->payload[7], // uvi
                    adv_data->payload[8], adv_data->payload[9], adv_data->payload[10], adv_data->payload[11], // ir
                    adv_data->payload[12], adv_data->payload[13], adv_data->payload[14], adv_data->payload[15], // temp
                    adv_data->payload[16], adv_data->payload[17], // humidity and moisture
                    _l, _u, _i, _t, _h, _m); // non hex values
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
    guardener_adv_data.adv_len = 22;
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
        .humidity = 0x11,
        .moisture = 0x22
    };
    strncpy((char *)guardener_adv_data.payload, (char *)&app_data, sizeof(app_data));

    // total length of advertising data
    guardener_adv_data.data_size = 1 + guardener_adv_data.adv_len;

    // Provide the BT stack the constructed advertisement packet
    sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle, 0, guardener_adv_data.data_size, (const uint8_t*)&guardener_adv_data);
    app_assert_status(sc);
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

    // reinitialize the advertiser
    guardener_init_ble_advertiser();

    /**
     * Acquire all the data to be broadcast
     */

    // Acquire Si1145's Readings
    float lux, uvi, ir;
    if(si1145_get_lux_uvi_ir(&lux, &uvi, &ir, 15) != SL_STATUS_OK)
    {
        app_log_error("Failed to acquire lux, uvi, and ir readings");
        app_log_nl();
        while(true); // crash here
    }
    else if(!calibrating)
    {
        app_log_info("lux=%0.2lf, uvi=%0.2lf, ir=%0.2lf", lux, uvi, ir);
    }

    // Acquire BME280's Readings
    float temps, humid;
    if(sl_bme280_force_get_readings(&temps, NULL, &humid) != SL_STATUS_OK)
    {
        app_log_error("Failed to acquire temperature, pressure, and humidity readings");
        app_log_nl();
        while(true); // crash here
    }
    else if(!calibrating)
    {
        app_log_info("temps=%0.2lf C, humid=%0.2lf %%", temps, humid);
        app_log_nl();
    }

    uint8_t humid_lvl = sl_bme280_convert_bme2RH(humid);
    if (humid_lvl == (uint8_t)-1)
    {
        app_log_error("Failed to scale BME280's humidity reading. Got \"%d\"", humid_lvl);
        app_log_nl();
    }

    // Acquire Moisture Sensor's millivolt Readings
    uint32_t mvolts = ms_get_millivolts();
    if(mvolts == (uint32_t)-1)
    {
        app_log_error("Failed to acquire moisture sensor value");
        app_log_nl();
        while(true); // crash here
    }
    else if((!calibrating) && (curr_cal_state != CAL_OK))
    {
        // if calibrated, can be % instead of mV
        app_log_info("moisture mvolts=%lu mV", mvolts);
        app_log_nl();
    }
    
    // Acquire Moisture Sensor's Percent Saturation Readings
    moisture_get_cal_state(&curr_cal_state);
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
    update_adv_packet(&guardener_adv_data, lux, uvi, ir, temps, humid_lvl, moisture_lvl);
    if (sl_bt_legacy_advertiser_set_data(advertising_set_handle, 0, guardener_adv_data.data_size, (const uint8_t*)&guardener_adv_data) != SL_STATUS_OK)
    {
        app_log_error("Failed to set advertising data");
        app_log_nl();
    }

    // start the advertisement with the new packet
    app_assert_status(sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable));

    /**
     * Wait 5 minutes - time it takes to collect the data before allowing the
     * application loop to trigger sending another advertisement packet.
     */
//    sl_sleeptimer_delay_millisecond(5000); // test wait
    for (int i = 0; i<4; i++)
    {
        sl_sleeptimer_delay_millisecond(65535 /*max sleep interval*/);
    }
    sl_sleeptimer_delay_millisecond(37860 /*remaining amount of 300000 ms*/);

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
