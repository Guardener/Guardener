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

// Moisture sensor handle.
static moisture_sensor_cfg_t ms_cfg = {0};
uint8_t adc_buff[8] = {0};

/******************************************************************************
 * Set up a custom advertisement package according to iBeacon specifications.
 * The advertisement package is 30 bytes long.
 * See the iBeacon specification for further details.
 *****************************************************************************/
static void
bcn_setup_adv_Guardenering(void);

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

    // PWM initialized by sl_pwm driver via TIMER1's CCO using PF7
    sl_pwm_set_duty_cycle(&sl_pwm_500k, 50);
    sl_pwm_start(&sl_pwm_500k);

    // Initialize Moisture Sensor
    uint32_t buff_sz = 8;
    moisture_sensor_cfg_t cfg = { // @formatter:off
        .vdac = VDAC0,                  /** VDAC Register Declaration */
        .dac_base_init = {
            .mainCalibration = true,    /** Use main output path calibration values. */
            .asyncClockMode = true,     /** Clock source for synchronous mode is HFPERCLK */
            .warmupKeepOn = false,      /** Turn off between sample off conversions.*/
            .refresh = vdacRefresh64,   /** Refresh every 8th cycle. */
            .prescaler = 0,             /** No prescaling. */
            .reference = vdacRefAvdd,   /** AVDD power reference. */
            .ch0ResetPre = false,       /** Do not reset prescaler on CH 0 start. */
            .outEnablePRS = true,       /** Enable PRS control of output driver */
            .sineEnable = true,         /** Enable sine wave generation mode. */
            .diff = false               /** Single ended mode. */
        },
        .dac0_init = {
            .enable = false,                /** Leave channel disabled when initialization is done. */
            .prsSel = vdacPrsSelCh0,        /** PRS CH 0 triggers conversion. */
            .prsAsync = true,               /** Treat PRS channel as a synchronous signal. */
            .trigMode = vdacTrigModePrs,    /** Select trigger as PRS. */
            .sampleOffMode = false          /** Channel conversion set to continuous. */
        },
        .dac1_init = {
            .enable = false,                /** Leave channel disabled when initialization is done. */
            .prsSel = vdacPrsSelCh1,        /** Select PRS channel 1 for DAC channel 1 */
            .prsAsync = true,               /** Treat PRS channel as a synchronous signal. */
            .trigMode = vdacTrigModePrs,    /** Select trigger as PRS. */
            .sampleOffMode = false          /** Channel conversion set to continuous. */
        },
        /* ADC Specific Configurations */
        .adc_clk_src = cmuClock_HFPER,      /**< Peripheral clock to use */
        .adc_osc_type = cmuOsc_AUXHFRCO,    /**< Oscillator type */
        .tgt_freq = cmuAUXHFRCOFreq_4M0Hz,  /**< Target frequency of ADC */
        .adc = ADC0,                        /**< ADC instance */
        .em_mode = adcEm2ClockOnDemand,     /**< Enable or disable EM2 ability */
        .adc_channel = adcPosSelAPORT2YCH22, /**< ADC channel PF6 */
        .ref_volts = adcRefVDD,             // TODO: verify we can do adcRefVDD
        .acq_time = adcAcqTime4,            /**< Acquisition time (in ADC clock cycles) */
        .prs_chan = adcPRSSELCh0,           /**< PRS Channel to use */
        .captures_per_sample = 2,           /**< Select single channel Data Valid level. SINGLE IRQ is set when (DVL+1) number of single channels have been converted and their results are available in the Single FIFO. */

        /* DMA Specific Configurations */
        .dma_clk_src = cmuClock_LDMA,       /**< Peripheral clock to use DMA */
        .dma_channel = 0,                   /**< DMA channel */
        .dma_trig = ldmaPeripheralSignal_ADC0_SINGLE, /**< What signal triggers the DMA to start */
        .dest_buff = adc_buff,              /**< Buffer were the ADC samples will be stored */
        .buff_size = buff_sz,               /**< Size of the buffer */

        /* LETimer Specific Configurations */
        .let_osc_type = cmuOsc_LFRCO,       /**< Oscillator type */
        .let_clk_src = cmuClock_LFA,        /**< Peripheral clock to use DMA */
        .letimer = LETIMER0,                /**< LETimer Peripheral to use */
        .delay_ms = 100,                    /**< Time to wait till triggering ADC reading (ms) */
    };  // @formatter:on
    ms_cfg = cfg;
    init_moisture_sensor(&ms_cfg);
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

    // Start BME280 Test
    float temps = 0.0, humid = 0.0;
    if(sl_bme280_force_get_readings(&temps, NULL, &humid) != SL_STATUS_OK)
    {
        app_log_error("Failed to acquire temperature, pressure, and humidity readings \r\n");
        while(true); // crash here
    }

    lux = 0.0, uvi = 0.0, ir = 0.0, temps = 0.0, humid = 0.0;

    int idx = 50; // Too lazy right now to add a real wait/delay
    do
    {
        app_log_info("ADC samples current value is: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X", adcBuffer[0],
                     adcBuffer[1], adcBuffer[2], adcBuffer[3], adcBuffer[4], adcBuffer[5], adcBuffer[6], adcBuffer[7]);
    } while(idx--);

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
