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
#include <app.h>
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "sl_status.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_timer.h"
#include "app_log.h"
#include "app_assert.h"
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#ifdef SL_CATALOG_CLI_PRESENT
#include "sl_cli.h"
#endif // SL_CATALOG_CLI_PRESENT
#include "sl_sensor_rht.h"
#include "sl_health_thermometer.h"
#include <stdbool.h>

#include "si1145.h"
#include "sl_pwm.h"
#include "moisture_sensor.h"
#include "bme280.h"

// Connection handle.
static uint8_t app_connection = 0;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// Button state.
static volatile bool app_btn0_pressed = false;

// Periodic timer handle.
static sl_simple_timer_t app_periodic_timer;

// PWM handle.
static sl_pwm_instance_t pwm_500k = {0};

// Moisture sensor handle.
static moisture_sensor_cfg_t ms_cfg = {0};
volatile uint8_t adcBuffer[8];

// Periodic timer callback.
static void app_periodic_timer_cb(sl_simple_timer_t* timer, void* data);
////////////////////////////////////////////////////////////////////////////////

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
    sl_status_t sc;
    app_log_init();
    app_log_info("App Initialization\r\n");

//    // Init temperature sensor.
//    sc = sl_sensor_rht_init();
//    if(sc != SL_STATUS_OK)
//    {
//        app_log_warning("Failed to init Si7021 RH/Temp sensor");
//        app_log_nl();
//    }
//
//    sc = si1145_init(I2C0);
//    if(sc != SL_STATUS_OK)
//    {
//        app_log_error("Failed to initialize the Si1145 sensor\r\n");
//        while(true); // crash here
//    }
//    fflush(stdout);

    // TODO: Resolve port issues; When PWM enabled, Si1145 fails.

//    // Initialize 500kHz PWM signal out of GPIO D10
//    // @formatter:off
//    sl_pwm_instance_t fivek_cfg = {
//            .timer = TIMER0,
//            .channel = 0,       /**< TIMER channel */
//            .port = gpioPortD,  /**< GPIO port */
//            .pin = 10,          /**< GPIO pin */
//            .location = TIMER_ROUTELOC0_CC0LOC_LOC15 /**< GPIO location */
//        };
//        // @formatter:on
//    pwm_500k = fivek_cfg;
//    sl_pwm_config_t pwm_cfg = {.frequency = 500000, .polarity = PWM_ACTIVE_HIGH};
//
//    sc = sl_pwm_init(&pwm_500k, &pwm_cfg);
//    if(sc != SL_STATUS_OK)
//    {
//        app_log_error("Failed to initialize the Si1145 sensor\n");
//        while(true); // crash here
//    }
//    else
//    {
//        sl_pwm_set_duty_cycle(&pwm_500k, 50);
//        // Start Moisture Sensor Test
//        sl_pwm_start(&pwm_500k);
//    }
//
//    // Initialize Moisture Sensor
//    uint32_t buff_sz = 8;
//    // @formatter:off
//    moisture_sensor_cfg_t cfg = {
//            /* ADC Specific Configurations */
//            .adc_clk_src = cmuClock_HFPER,      /**< Peripheral clock to use */
//            .adc_osc_type = cmuOsc_AUXHFRCO,    /**< Oscillator type */
//            .tgt_freq = cmuAUXHFRCOFreq_4M0Hz,  /**< Target frequency of ADC */
//            .adc = ADC0,                        /**< ADC instance */
//            .em_mode = adcEm2ClockOnDemand,     /**< Enable or disable EM2 ability */
//            .adc_channel = adcPosSelAPORT2XCH9, /**< ADC channel */
//            .ref_volts = adcRef2V5,             // TODO: verify we can do adcRefVDD
//            .acq_time = adcAcqTime4,            /**< Acquisition time (in ADC clock cycles) */
//            .prs_chan = adcPRSSELCh0,           /**< PRS Channel to use */
//            .captures_per_sample = 2,           /**< Select single channel Data Valid level. SINGLE IRQ is set when (DVL+1) number of single channels have been converted and their results are available in the Single FIFO. */
//
//            /* DMA Specific Configurations */
//            .dma_clk_src = cmuClock_LDMA,       /**< Peripheral clock to use DMA */
//            .dma_channel = 0,                   /**< DMA channel */
//            .dma_trig = ldmaPeripheralSignal_ADC0_SINGLE, /**< What signal triggers the DMA to start */
//            .dest_buff = adcBuffer,             /**< Buffer were the ADC samples will be stored */
//            .buff_size = buff_sz,               /**< Size of the buffer */
//
//            /* LETimer Specific Configurations */
//            .let_osc_type = cmuOsc_LFRCO,       /**< Oscillator type */
//            .let_clk_src = cmuClock_LFA,        /**< Peripheral clock to use DMA */
//            .letimer = LETIMER0,                /**< LETimer Peripheral to use */
//            .delay_ms = 100,                    /**< Time to wait till triggering ADC reading (ms) */
//    };
//        // @formatter:on
//    ms_cfg = cfg;
//    init_moisture_sensor(&ms_cfg);

    bme280_init_t init_cfg = {
        .opt_mode = BEM280_CMD_CTRL_MEAS_MODE_NORMAL,
        .acqu_intvl = BME280_REG_CONFIG_T_SB_0_5_MS,
        .temp_samp_rate = BEM280_CMD_CTRL_MEAS_OSRS_T_16X_SAMPLING,
        .hum_samp_rate = BME280_REG_CTRL_HUM_OSRS_H_16X_SAMPLING,
        .pres_samp_rate = BEM280_CMD_CTRL_MEAS_OSRS_P_16X_SAMPLING,
        .filter = BME280_REG_CONFIG_FILTER_OFF
    };
    bme280_init(I2C0, init_cfg);
}

#ifndef SL_CATALOG_KERNEL_PRESENT
/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
    // Start Si1145 Test
//    float lux = 0.0, uvi = 0.0;
//    if(si1145_get_lux_uvi(I2C0, &lux, &uvi) == SL_STATUS_OK)
//    {
//        app_log_info("Acquired Lux and UVI: %f & %f \r\n", lux, uvi);
//        fflush(stdout);
//    }
//    else
//    {
//        app_log_error("Failed to acquire lux and uvi readings \r\n");
//        fflush(stdout);
//        while(true); // crash here
//    }
    // TODO: Fix Lux/UV init/math... values are always the same

//    int idx = 50; // Too lazy right now to add a real wait/delay
//    do
//    {
//        app_log_info("ADC samples current value is: 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X\r\n", adcBuffer[0],
//                     adcBuffer[1], adcBuffer[2], adcBuffer[3], adcBuffer[4], adcBuffer[5], adcBuffer[6], adcBuffer[7]);
//    } while(idx--);
//    app_log_info("Moisture Sensor Demo Done!\r\n");
//    fflush(stdout);

    // Samples should happen every .5 ms
    sl_sleeptimer_delay_millisecond(10);
    float temps = 0.0, press = 0.0, humid = 0.0;
    temps = bme280_get_temperature();
    press = bme280_get_pressure();
    humid = bme280_get_humidity();
    app_log_info("temp=%f, press=%f, humid=%f", temps, press, humid);
    app_log_info("BME280 Test... Success?");
    return;
}
#endif // SL_CATALOG_KERNEL_PRESENT

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

    switch(SL_BT_MSG_ID(evt->header))
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

            app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                         address_type ? "static random" : "public device", address.addr[5], address.addr[4],
                         address.addr[3], address.addr[2], address.addr[1], address.addr[0]);

            // Create an advertising set.
            sc = sl_bt_advertiser_create_set(&advertising_set_handle);
            app_assert_status(sc);

            // Generate data for advertising
            sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle, sl_bt_advertiser_general_discoverable);
            app_assert_status(sc);

            // Set advertising interval to 100ms.
            sc = sl_bt_advertiser_set_timing(advertising_set_handle, 160, // min. adv. interval (milliseconds * 1.6)
                                             160, // max. adv. interval (milliseconds * 1.6)
                                             0,   // adv. duration
                                             0);  // max. num. adv. events
            app_assert_status(sc);

            // Start advertising and enable connections.
            sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
            app_assert_status(sc);
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

            ///////////////////////////////////////////////////////////////////////////
            // Add additional event handlers here as your application requires!      //
            ///////////////////////////////////////////////////////////////////////////

            // -------------------------------
            // Default event handler.
        default:
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
 * Health Thermometer - Temperature Measurement
 * Indication changed callback
 *
 * Called when indication of temperature measurement is enabled/disabled by
 * the client.
 *****************************************************************************/
void sl_bt_ht_temperature_measurement_indication_changed_cb(uint8_t connection,
                                                            sl_bt_gatt_client_config_flag_t client_config)
{
    sl_status_t sc;
    app_connection = connection;
    // Indication or notification enabled.
    if(sl_bt_gatt_disable != client_config)
    {
        // Start timer used for periodic indications.
        sc = sl_simple_timer_start(&app_periodic_timer,
        SL_BT_HT_MEASUREMENT_INTERVAL_SEC * 1000,
                                   app_periodic_timer_cb,
                                   NULL,
                                   true);
        app_assert_status(sc);
        // Send first indication.
        app_periodic_timer_cb(&app_periodic_timer, NULL);
    }
    // Indications disabled.
    else
    {
        // Stop timer used for periodic indications.
        (void)sl_simple_timer_stop(&app_periodic_timer);
    }
}

/**************************************************************************//**
 * Simple Button
 * Button state changed callback
 * @param[in] handle Button event handle
 *****************************************************************************/
void sl_button_on_change(const sl_button_t* handle)
{
    // Button pressed.
    if(sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED)
    {
        if(&sl_button_btn0 == handle)
        {
            app_btn0_pressed = true;
        }
    }
    // Button released.
    else if(sl_button_get_state(handle) == SL_SIMPLE_BUTTON_RELEASED)
    {
        if(&sl_button_btn0 == handle)
        {
            app_btn0_pressed = false;
        }
    }
}

/**************************************************************************//**
 * Timer callback
 * Called periodically to time periodic temperature measurements and indications.
 *****************************************************************************/
static void app_periodic_timer_cb(sl_simple_timer_t* timer, void* data)
{
    (void)data;
    (void)timer;
    sl_status_t sc;
    int32_t temperature = 0;
    uint32_t humidity = 0;
    float tmp_c = 0.0;
    // float tmp_f = 0.0;

    // Measure temperature; units are % and milli-Celsius.
    sc = sl_sensor_rht_get(&humidity, &temperature);
    if(SL_STATUS_NOT_INITIALIZED == sc)
    {
        app_log_info("Relative Humidity and Temperature sensor is not initialized.");
        app_log_nl();
    }
    else if(sc != SL_STATUS_OK)
    {
        app_log_warning("Invalid RHT reading: %lu %ld\n", humidity, temperature);
    }

    // button 0 pressed: overwrite temperature with -20C.
    if(app_btn0_pressed)
    {
        temperature = -20 * 1000;
    }

    tmp_c = (float)temperature / 1000;
    app_log_info("Temperature: %5.2f C\n", tmp_c);
    // Send temperature measurement indication to connected client.
    sc = sl_bt_ht_temperature_measurement_indicate(app_connection, temperature,
    false);
    // Conversion to Fahrenheit: F = C * 1.8 + 32
    // tmp_f = (float)(temperature*18+320000)/10000;
    // app_log_info("Temperature: %5.2f F\n", tmp_f);
    // Send temperature measurement indication to connected client.
    // sc = sl_bt_ht_temperature_measurement_indicate(app_connection,
    //                                                (temperature*18+320000)/10,
    //                                                true);
    if(sc)
    {
        app_log_warning("Failed to send temperature measurement indication\n");
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
