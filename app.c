/*******************************************************************************
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

#include "sl_bluetooth.h"
#include "app_assert.h"
#include "gatt_db.h"
#include "app.h"
#include "sl_i2cspm_si1145_config.h"
#include "si1145.h"
#include "sl_i2cspm_bme280_config.h"
#include "sl_bme280.h"
#include "sl_pwm_instances.h"
#include "moisture_sensor.h"
#include "sl_button.h"

#ifndef app_log_error
#define app_log_error(...) do { /* nop */ } while (0)
#define app_log_debug(...) do { /* nop */ } while (0)
#define app_log_info(...) do { /* nop */ } while (0)
#endif

// Macros.
#define UINT16_TO_BYTES(n) ((uint8_t)(n)), ((uint8_t)((n) >> 8))
#define UINT16_TO_BYTE0(n) ((uint8_t)(n))
#define UINT16_TO_BYTE1(n) ((uint8_t)((n) >> 8))

#define SIGNAL_LOSS_AT_1_M_IN_DBM 41 // The Guardener's measured RSSI at 1 m

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

/******************************************************************************
 * Set up a custom advertisement package according to iBeacon specifications.
 * The advertisement package is 30 bytes long.
 * See the iBeacon specification for further details.
 *****************************************************************************/
static void bcn_setup_adv_Guardenering(void);

/******************************************************************************
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
    // Init Si1145
    si1145_cfg_t init = { // @formatter:off
        .i2cspm = SL_I2CSPM_SI1145_PERIPHERAL, // I2C peripheral initialized already
        .uv = true,     // enable use of UV
        .temp = true,   // enable use of temperature
        .proxy = false, // disable use of proximity sensor
        .irq = false,   // disable interrupts
        .forced = true  // disable auto mode, use forced readings
    };  // @formatter:on
    sl_status_t sc = si1145_init(init);
    if(sc != SL_STATUS_OK)
    {
        app_log_error("Failed to initialize the Si1145 sensor\r\n");
        while(true); // crash here
    }

    // Init BME280
    struct bme280_settings init_cfg = { // @formatter:off
        .osr_p = BME280_OVERSAMPLING_1X,    		/*< pressure oversampling */
        .osr_t = BME280_OVERSAMPLING_1X,    		/*< temperature oversampling */
        .osr_h = BME280_NO_OVERSAMPLING,    		/*< humidity oversampling */
        .filter = BME280_FILTER_COEFF_OFF,  		/*< filter coefficient */
        .standby_time = BME280_STANDBY_TIME_0_5_MS  /*< standby time */
    };  // @formatter:on
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
    };  // @formatter:on
    init_moisture_sensor(&ms_cfg);
}

/******************************************************************************
 * Overrides sl_button's IRQ callback (which is empty) with ours
 *****************************************************************************/
static bool interrupt_triggered = false;
void sl_button_on_change(const sl_button_t *handle)
{
    (void)handle; // the button pressed
    interrupt_triggered = true;
}

/******************************************************************************
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
    // Start Si1145 Test
    float lux = 0.0, uvi = 0.0, ir = 0.0;
    if(si1145_get_lux_uvi_ir(&lux, &uvi, &ir, 15) != SL_STATUS_OK)
    {
        app_log_error("Failed to acquire lux, uvi, and ir readings \r\n");
        while(true); // crash here
    }
    lux = 0.0, uvi = 0.0, ir = 0.0; // debugger stop here to verify

    // Start BME280 Test
    float temps = 0.0, humid = 0.0;
    if(sl_bme280_force_get_readings(&temps, NULL, &humid) != SL_STATUS_OK)
    {
        app_log_error("Failed to acquire temperature, pressure, and humidity readings \r\n");
        while(true); // crash here
    }
    temps = 0.0, humid = 0.0; // debugger stop here to verify

    uint32_t mvolts = ms_get_millivolts();
    if(mvolts == (uint32_t)-1)
    {
        app_log_error("Failed to acquire moisture sensor value \r\n");
        while(true); // crash here
    }
    mvolts = 0; // debugger stop here to verify

    if (interrupt_triggered)
    {
        interrupt_triggered = false; // debugger stop here to verify
        app_log_info("User Interface Button has been pressed %lu times", pressed_count);
    }

    return;
}

/******************************************************************************
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  int16_t ret_power_min, ret_power_max;
  switch (SL_BT_MSG_ID(evt->header))
  {
  // -------------------------------
  // This event indicates the device has started and the radio is ready.
  // Do not call any stack command before receiving this boot event!
  case sl_bt_evt_system_boot_id:
    // Set 0 dBm maximum Transmit Power.
    sc = sl_bt_system_set_tx_power(SL_BT_CONFIG_MIN_TX_POWER, 0,
                                   &ret_power_min, &ret_power_max);
    app_assert_status(sc);
    (void)ret_power_min;
    (void)ret_power_max;
    // Initialize iBeacon ADV data.
    bcn_setup_adv_Guardenering();
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

static void
bcn_setup_adv_Guardenering(void)
{
  sl_status_t sc;
  int16_t support_min;
  int16_t support_max;
  int16_t set_min;
  int16_t set_max;
  int16_t rf_path_gain;
  int16_t calculated_power;

  PACKSTRUCT(struct
             {
               uint8_t flags_len;    // Length of the Flags field.
               uint8_t flags_type;   // Type of the Flags field.
               uint8_t flags;        // Flags field.
               uint8_t mandata_len;  // Length of the Manufacturer Data field.
               uint8_t mandata_type; // Type of the Manufacturer Data field.
               uint8_t comp_id[2];   // Company ID field.
               uint8_t beac_type[2]; // Beacon Type field.
               uint8_t uuid[16];     // 128-bit Universally Unique Identifier (UUID). The UUID is an identifier for the company using the Guardener.
               uint8_t maj_num[2];   // Beacon major number. Used to group related Guardeners.
               uint8_t min_num[2];   // Beacon minor number. Used to specify individual Guardeners within a group.
               int8_t tx_power;      // The Beacon's measured RSSI at 1 meter distance in dBm. See the iBeacon specification for measurement guidelines.
             })
  bcn_Guardener_adv_data =
      {
          // Flag bits - See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags.
          2,           // Length of field.
          0x01,        // Type of field.
          0x04 | 0x02, // Flags: LE General Discoverable Mode, BR/EDR is disabled.

          // Manufacturer specific data.
          26,   // Length of field.
          0xFF, // Type of field.

          // The first two data octets shall contain a company identifier code from
          // the Assigned Numbers - Company Identifiers document.
          // 0x004C = Apple
          {UINT16_TO_BYTES(0x004C)},

          // Beacon type.
          // 0x0215 is iBeacon.
          {UINT16_TO_BYTE1(0x0215), UINT16_TO_BYTE0(0x0215)},

          // 128 bit / 16 byte UUID
          {0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, 0xB0, 0x60, 0xD0,
           0xF5, 0xA7, 0x10, 0x96, 0xE0},

          // Beacon major number.
          // Set to 34987 and converted to correct format.
          {UINT16_TO_BYTE1(34987), UINT16_TO_BYTE0(34987)},

          // Beacon minor number.
          // Set as 1025 and converted to correct format.
          {UINT16_TO_BYTE1(1025), UINT16_TO_BYTE0(1025)},

          // A dummy value which will be eventually overwritten
          0};

  // Create an advertising set.
  sc = sl_bt_advertiser_create_set(&advertising_set_handle);
  app_assert_status(sc);

  sc = sl_bt_system_get_tx_power_setting(&support_min, &support_max, &set_min,
                                         &set_max, &rf_path_gain);
  app_assert_status(sc);

  calculated_power = (set_max > 0) ? (set_max + 5) / 10 : (set_max - 5) / 10;
  calculated_power = calculated_power - SIGNAL_LOSS_AT_1_M_IN_DBM;

  if (calculated_power > INT8_MAX)
  {
    bcn_Guardener_adv_data.tx_power = INT8_MAX;
  }
  else if (calculated_power < INT8_MIN)
  {
    bcn_Guardener_adv_data.tx_power = INT8_MIN;
  }
  else
  {
    bcn_Guardener_adv_data.tx_power = calculated_power;
  }

  // Set custom advertising data.
  sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle, 0,
                                        sizeof(bcn_Guardener_adv_data),
                                        (uint8_t *)(&bcn_Guardener_adv_data));
  app_assert_status(sc);

  // Set advertising parameters. 100ms advertisement interval.
  sc = sl_bt_advertiser_set_timing(advertising_set_handle, 160, // min. adv. interval (milliseconds * 1.6)
                                   160,                         // max. adv. interval (milliseconds * 1.6)
                                   0,                           // adv. duration
                                   0);                          // max. num. adv. events
  app_assert_status(sc);

  // Start advertising in user mode and disable connections.
  sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                     sl_bt_advertiser_non_connectable);
  app_assert_status(sc);
}
