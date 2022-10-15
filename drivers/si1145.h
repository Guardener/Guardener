/*******************************************************************************
 * @file si1145.h
 *
 *  Created on: Oct 10, 2022
 *      Author: Caleb Provost
 *
 * @brief Silicon Labs' Proximity/UV/Ambient Light Sensor IC w/I2C interface
 * @see   https://www.silabs.com/documents/public/data-sheets/Si1145-46-47.pdf
 * @note  Code structure created following existing SiLabs driver methodology
 *
 ******************************************************************************/

#ifndef DRIVERS_SI1145_H_
#define DRIVERS_SI1145_H_

//#include <stdint.h>
#include "sl_i2cspm.h"
#include "sl_status.h"

#if ENABLE_PROXIMITY == 1
#define INIT_PROX
#endif

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief   Register interface and implementation details
 ******************************************************************************/
#define SI1145_HW_KEY_VAL   (0x17) /**< The system must write this value to register HW_KEY for proper Si114x operation. */
#define SI1145_PART_ID_VAL  (0x45) /**< Read only value of part id, set to 0x45 by default for the Si1145 */

/**************************************************************************//**
 * @brief Si1145 register definitions
 ******************************************************************************/
#define SI1145_REG_PART_ID              (0x00)  
#define SI1145_REG_REV_ID               (0x01)  
#define SI1145_REG_SEQ_ID               (0x02)  
#define SI1145_REG_INT_CFG              (0x03)  /**< Bit 0=INT_OE */
#define SI1145_REG_IRQ_ENABLE           (0x04)  /**< Bit 4=PS3_IE; 3=PS2_IE; 2=PS1_IE; 0=ALS_IE */
#define SI1145_REG_HW_KEY               (0x07)
#define SI1145_REG_MEAS_RATE0           (0x08)  
#define SI1145_REG_MEAS_RATE1           (0x09)  
#define SI1145_REG_RESERVED0            (0x0A)
#define SI1145_REG_RESERVED1            (0x0B)
#define SI1145_REG_RESERVED2            (0x0C)
#define SI1145_REG_RESERVED3            (0x0D)
#define SI1145_REG_RESERVED4            (0x0E)
#define SI1145_REG_PS_LED21             (0x0F)  /**< Bits [7-4]=LED2_I; [3-0]=LED1_I */
#define SI1145_REG_PS_LED3              (0x10)  /**< Bits [3-0]=LED3_I */
#define SI1145_REG_RESERVED5            (0x11)
#define SI1145_REG_RESERVED6            (0x12)
#define SI1145_REG_UCOEF0               (0x13)  
#define SI1145_REG_UCOEF1               (0x14)  
#define SI1145_REG_UCOEF2               (0x15)  
#define SI1145_REG_UCOEF3               (0x16)  
#define SI1145_REG_PARAM_WR             (0x17)  
#define SI1145_REG_COMMAND              (0x18)  
#define SI1145_REG_RESPONSE             (0x20)  
#define SI1145_REG_IRQ_STATUS           (0x21)  /**< Bit 5=CMD_INT; 4=PS3_INT; 3=PS2_INT; 2=PS1_INT; [1-0]=ALS_INT */
#define SI1145_REG_ALS_VIS_DATA0        (0x22)  
#define SI1145_REG_ALS_VIS_DATA1        (0x23)  
#define SI1145_REG_ALS_IR_DATA0         (0x24)  
#define SI1145_REG_ALS_IR_DATA1         (0x25)  
#define SI1145_REG_PS1_DATA0            (0x26)  
#define SI1145_REG_PS1_DATA1            (0x27)  
#define SI1145_REG_PS2_DATA0            (0x28)  
#define SI1145_REG_PS2_DATA1            (0x29)  
#define SI1145_REG_PS3_DATA0            (0x2A)  
#define SI1145_REG_PS3_DATA1            (0x2B)  
#define SI1145_REG_AUX_DATA0_UVINDEX0   (0x2C)  
#define SI1145_REG_AUX_DATA1_UVINDEX1   (0x2D)  
#define SI1145_REG_PARAM_RD             (0x2E)  
#define SI1145_REG_CHIP_STAT            (0x30)  /**< Bit 2=RUNNING; 1=SUSPEND; 0=SLEEP */
#define SI1145_REG_ANA_IN_KEY1          (0x3B)
#define SI1145_REG_ANA_IN_KEY2          (0x3C)
#define SI1145_REG_ANA_IN_KEY3          (0x3D)
#define SI1145_REG_ANA_IN_KEY4          (0x3E)

#define SI1145_RSP0_CHIPSTAT_MASK       (0x07)
#define SI1145_RSP0_COUNTER_MASK        (0xF8)
#define SI1145_RSP0_SLEEP               (0x01)

/**************************************************************************//**
 * @brief Si1145 Register Parameters; Parameters explicitly defined for registers
 ******************************************************************************/
#define SI1145_REG_INT_CFG_PARAM_INT_OE                 (0x01)  /**< Readability macro for the enabling of INT_OE */
#define SI1145_REG_IRQ_ENABLE_PARAM_IRQ_PS3_IE          (0x10)  /**< Readability macro for the enabling of PS3_IE */
#define SI1145_REG_IRQ_ENABLE_PARAM_IRQ_PS2_IE          (0x08)  /**< Readability macro for the enabling of PS2_IE */
#define SI1145_REG_IRQ_ENABLE_PARAM_IRQ_PS1_IE          (0x04)  /**< Readability macro for the enabling of PS1_IE */
#define SI1145_REG_IRQ_ENABLE_PARAM_IRQ_ALS_IE          (0x01)  /**< Readability macro for the enabling of ALS_IE */
#define SI1145_REG_IRQ_STATUS_PARAM_IRQ_STATUS_CMD_INT  (0x20)  /**< Readability macro for checking the IRQ Status */
#define SI1145_REG_IRQ_STATUS_PARAM_IRQ_STATUS_PS3_INT  (0x10)  /**< Readability macro for checking the IRQ Status */
#define SI1145_REG_IRQ_STATUS_PARAM_IRQ_STATUS_PS2_INT  (0x08)  /**< Readability macro for checking the IRQ Status */
#define SI1145_REG_IRQ_STATUS_PARAM_IRQ_STATUS_PS1_INT  (0x04)  /**< Readability macro for checking the IRQ Status */
#define SI1145_REG_IRQ_STATUS_PARAM_IRQ_STATUS_ALS_INT  (0x03)  /**< Readability macro for checking the IRQ Status */

/**************************************************************************//**
 * @brief Si1145 Parameters are located in internal memory and are not directly
 *           addressable over I2C. They must be indirectly accessed using the
 *           PARAM_QUERY and PARAM_SET commands
 ******************************************************************************/
#define SI1145_PARAM_I2C_ADDR               (0x00)  /**< I2C Address */
#define SI1145_PARAM_CHLIST                 (0x01)  /**< EN_UV EN_AUX EN_ALS_IR EN_ALS_VIS - EN_PS3 EN_PS2 EN_PS1 */
#define SI1145_PARAM_PSLED12_SELECT         (0x02)  /**< PS2_LED - PS1_LED */
#define SI1145_PARAM_PSLED3_SELECT          (0x03)  /**< PS3_LED */
#define SI1145_PARAM_RESERVED1              (0x04)  /**< Reserved (always set to 0) */
#define SI1145_PARAM_PS_ENCODING            (0x05)  /**< PS3_ALIGN PS2_ALIGN PS1_ALIGN Reserved (always set to 0) */
#define SI1145_PARAM_ALS_ENCODING           (0x06)  /**< ALS_IR_ALI GN ALS_VIS_ ALIGN Reserved (always set to 0) */
#define SI1145_PARAM_PS1_ADCMUX             (0x07)  /**< PS1 ADC Input Selection */
#define SI1145_PARAM_PS2_ADCMUX             (0x08)  /**< PS2 ADC Input Selection */
#define SI1145_PARAM_PS3_ADCMUX             (0x09)  /**< PS3 ADC Input Selection */
#define SI1145_PARAM_PS_ADC_COUNTER         (0x0A)  /**< PS_ADC_REC Reserved (always set to 0) */
#define SI1145_PARAM_PS_ADC_GAIN            (0x0B)  /**< PS_ADC_GAIN */
#define SI1145_PARAM_PS_ADC_MISC            (0x0C)  /**< PS_RANGE - PS_ADC_ MODE */
#define SI1145_PARAM_RESERVED2              (0x0D)  /**< Reserved (do not modify from default setting of 0x02) */
#define SI1145_PARAM_ALS_IR_ADCMUX          (0x0E)  /**< ALS_IR_ADCMUX */
#define SI1145_PARAM_AUX_ADCMUX             (0x0F)  /**< AUX ADC Input Selection */
#define SI1145_PARAM_ALS_VIS_ADC_COUNTER    (0x10)  /**< VIS_ADC_REC Reserved (always set to 0) */
#define SI1145_PARAM_ALS_VIS_ADC_GAIN       (0x11)  /**< ALS_VIS_ADC_GAIN */
#define SI1145_PARAM_ALS_VIS_ADC_MISC       (0x12)  /**< Reserved (always set to 0) VIS_RANGE Reserved (always set to 0) */
#define SI1145_PARAM_RESERVED3              (0x13)  /**< Reserved (do not modify from default setting of 0x40) */
#define SI1145_PARAM_RESERVED4              (0x14)  /**< Reserved (do not modify from default setting of 0x00) */
#define SI1145_PARAM_RESERVED5              (0x15)  /**< Reserved (do not modify from default setting of 0x00) */
#define SI1145_PARAM_RESERVED6              (0x1B)  /**< Reserved (do not modify from default setting of 0x00) */
#define SI1145_PARAM_LED_REC                (0x1C)  /**< LED recovery time ALS_IR_ADC_COUNTER 0x1D - IR_ADC_REC Reserved (always set to 0) */
#define SI1145_PARAM_ALS_IR_ADC_COUNTER     (0x1D)  /**< IR_ADC_REC */
#define SI1145_PARAM_ALS_IR_ADC_GAIN        (0x1E)  /**< ALS_IR_ADC_GAIN */
#define SI1145_PARAM_ALS_IR_ADC_MISC        (0x1F)  /**< Reserved (always set to 0) IR_RANGE Reserved (always set to 0) */

// For SI1145_PARAM_CHLIST
#define SI1145_PARAM_CHLIST_EN_UV               (0x80)  /**< Enables UV Index, data stored in AUX_DATA1[7:0] and AUX_DATA0[7:0] */
#define SI1145_PARAM_CHLIST_EN_AUX              (0x40)  /**< Enables Auxiliary Channel, data stored in AUX_DATA1[7:0] and AUX_DATA0[7:0]. */
#define SI1145_PARAM_CHLIST_EN_EN_ALS_IR        (0x20)  /**< Enables ALS IR Channel, data stored in ALS_IR_DATA1[7:0] and ALS_IR_DATA0[7:0]. */
#define SI1145_PARAM_CHLIST_EN_EN_ALS_VIS       (0x10)  /**< Enables ALS Visible Channel, data stored in ALS_VIS_DATA1[7:0] and ALS_VIS_DATA0[7:0]. */
#define SI1145_PARAM_CHLIST_EN_PS3              (0x04)  /**< Enables PS Channel 3, data stored in PS3_DATA1[7:0] and PS3_DATA0[7:0]. */
#define SI1145_PARAM_CHLIST_EN_PS2              (0x02)  /**< Enables PS Channel 2, data stored in PS2_DATA1[7:0] and PS2_DATA0[7:0]. */
#define SI1145_PARAM_CHLIST_EN_PS1              (0x01)  /**< Enables PS Channel 1, data stored in PS1_DATA1[7:0] and PS1_DATA0[7:0]. */
// For SI1145_PARAM_PSLED12_SELECT
#define SI1145_PARAM_PSLED12_SELECT_PS2_NONE    (0x00)  /**< No PS2 LED Drives Enabled */
#define SI1145_PARAM_PSLED12_SELECT_PS2_LED1    (0x10)  /**< PS2 LED1 Drive Enabled */
#define SI1145_PARAM_PSLED12_SELECT_PS2_LED2    (0x20)  /**< PS2 LED2 Drive Enabled (Si1146 and Si1147 only. Clear for Si1145) */
#define SI1145_PARAM_PSLED12_SELECT_PS2_LED3    (0x40)  /**< PS2 LED3 Drive Enabled (Si1147 only. Clear for Si1145 and Si1146) */
#define SI1145_PARAM_PSLED12_SELECT_PS1_NONE    (0x00)  /**< No PS1 LED Drives Enabled */
#define SI1145_PARAM_PSLED12_SELECT_PS1_LED1    (0x01)  /**< PS2 LED1 Drive Enabled */
#define SI1145_PARAM_PSLED12_SELECT_PS1_LED2    (0x02)  /**< PS2 LED2 Drive Enabled (Si1146 and Si1147 only. Clear for Si1145) */
#define SI1145_PARAM_PSLED12_SELECT_PS1_LED3    (0x04)  /**< PS2 LED3 Drive Enabled (Si1147 only. Clear for Si1145 and Si1146) */
// For SI1145_PARAM_PSLED3_SELECT
#define SI1145_PARAM_PSLED3_SELECT_PS1_NONE     (0x00)  /**< No PS1 LED Drives Enabled */
#define SI1145_PARAM_PSLED3_SELECT_PS1_LED1     (0x01)  /**< PS2 LED1 Drive Enabled */
#define SI1145_PARAM_PSLED3_SELECT_PS1_LED2     (0x02)  /**< PS2 LED2 Drive Enabled (Si1146 and Si1147 only. Clear for Si1145) */
#define SI1145_PARAM_PSLED3_SELECT_PS1_LED3     (0x04)  /**< PS2 LED3 Drive Enabled (Si1147 only. Clear for Si1145 and Si1146) */
// For SI1145_PARAM_PS_ENCODING
#define SI1145_PARAM_PS_ENCODING_PS3_ALIGN      (0x40) /**< When set, the ADC reports the least significant 16 bits of the 17-bit ADC when performing PS3 Measurement. Reports the 16 MSBs when cleared. */
#define SI1145_PARAM_PS_ENCODING_PS2_ALIGN      (0x20) /**< When set, the ADC reports the least significant 16 bits of the 17-bit ADC when performing PS2 Measurement. Reports the 16 MSBs when cleared. */
#define SI1145_PARAM_PS_ENCODING_PS1_ALIGN      (0x10) /**< When set, the ADC reports the least significant 16 bits of the 17-bit ADC when performing PS1 Measurement. Reports the 16 MSBs when cleared. */
// For SI1145_PARAM_ALS_ENCODING
#define SI1145_PARAM_ALS_ENCODING_ALS_IR_ALIGN  (0x20) /**< When set, the ADC reports the least significant 16 bits of the 17-bit ADC when performing ALS VIS Measurement. Reports the 16 MSBs when cleared. */
#define SI1145_PARAM_ALS_ENCODING_ALS_VIS_ALIGN (0x10) /**< When set, the ADC reports the least significant 16 bits of the 17-bit ADC when performing ALS IR Measurement. Reports the 16 MSBs when cleared. */
// For SI1145_PARAM_PS1_ADCMUX
#define SI1145_PARAM_PS1_ADCMUX_LARGE_IR        (0x03)  /**< Large IR Photodiode A separate "No Photodiode" measurement should be subtracted to arrive at Ambient IR reading. */
#define SI1145_PARAM_PS1_ADCMUX_SMALL_IR        (0x00)  /**< Small IR Photodiode A separate "No Photodiode" measurement should be subtracted to arrive at Ambient IR reading. */
#define SI1145_PARAM_PS1_ADCMUX_VISIBLE         (0x02)  /**< No Photodiode This is typically used as reference for reading ambient IR or visible light. */
#define SI1145_PARAM_PS1_ADCMUX_GND_VOLT        (0x25)  /**< GND voltage This is typically used as the reference for electrical measurements. */
#define SI1145_PARAM_PS1_ADCMUX_TEMP            (0x65)  /**< Temperature (Should be used only for relative temperature measurement. Absolute Temperature not guaranteed) A separate GND measurement should be subtracted from this reading. */
#define SI1145_PARAM_PS1_ADCMUX_VDD             (0x75)  /**< VDD voltage A separate GND measurement is needed to make the measurement meaningful. */
// For SI1145_PARAM_PS_ADC_COUNTER
// The recommended PS_ADC_REC value is the one's complement of PS_ADC_GAIN.
#define SI1145_PARAM_PS_ADC_COUNTER_1_CLK       (0x00) /**< 000: 1 ADC Clock (50 ns times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_PS_ADC_COUNTER_7_CLK       (0x01) /**< 001: 7 ADC Clock (350 ns times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_PS_ADC_COUNTER_15_CLK      (0x02) /**< 010: 15 ADC Clock (750 ns times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_PS_ADC_COUNTER_31_CLK      (0x03) /**< 011: 31 ADC Clock (1.55 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_PS_ADC_COUNTER_63_CLK      (0x04) /**< 100: 63 ADC Clock (3.15 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_PS_ADC_COUNTER_127_CLK     (0x05) /**< 101: 127 ADC Clock (6.35 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_PS_ADC_COUNTER_255_CLK     (0x06) /**< 110: 255 ADC Clock (12.75 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_PS_ADC_COUNTER_511_CLK     (0x07) /**< 111: 511 ADC Clock (25.55 us times 2ALS_VIS_ADC_GAIN) */
// For SI1145_PARAM_ALS_VIS_ADC_COUNTER
// The recommended VIS_ADC_REC value is the one's complement of ALS_VIS_ADC_GAIN
#define SI1145_PARAM_ALS_VIS_ADC_COUNTER_1_CLK   (0x00) /**< 000: 1 ADC Clock (50 ns times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_VIS_ADC_COUNTER_7_CLK   (0x01) /**< 001: 7 ADC Clock (350 ns times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_VIS_ADC_COUNTER_15_CLK  (0x02) /**< 010: 15 ADC Clock (750 ns times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_VIS_ADC_COUNTER_31_CLK  (0x03) /**< 011: 31 ADC Clock (1.55 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_VIS_ADC_COUNTER_63_CLK  (0x04) /**< 100: 63 ADC Clock (3.15 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_VIS_ADC_COUNTER_127_CLK (0x05) /**< 101: 127 ADC Clock (6.35 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_VIS_ADC_COUNTER_255_CLK (0x06) /**< 110: 255 ADC Clock (12.75 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_VIS_ADC_COUNTER_511_CLK (0x07) /**< 111: 511 ADC Clock (25.55 us times 2ALS_VIS_ADC_GAIN) */
// For SI1145_PARAM_ALS_IR_ADC_COUNTER
// The recommended IR_ADC_REC value is the one's complement of ALS_IR_ADC_GAIN.
#define SI1145_PARAM_ALS_IR_ADC_COUNTER_1_CLK   (0x00) /**< 000: 1 ADC Clock (50 ns times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_IR_ADC_COUNTER_7_CLK   (0x01) /**< 001: 7 ADC Clock (350 ns times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_IR_ADC_COUNTER_15_CLK  (0x02) /**< 010: 15 ADC Clock (750 ns times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_IR_ADC_COUNTER_31_CLK  (0x03) /**< 011: 31 ADC Clock (1.55 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_IR_ADC_COUNTER_63_CLK  (0x04) /**< 100: 63 ADC Clock (3.15 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_IR_ADC_COUNTER_127_CLK (0x05) /**< 101: 127 ADC Clock (6.35 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_IR_ADC_COUNTER_255_CLK (0x06) /**< 110: 255 ADC Clock (12.75 us times 2ALS_VIS_ADC_GAIN) */
#define SI1145_PARAM_ALS_IR_ADC_COUNTER_511_CLK (0x07) /**< 111: 511 ADC Clock (25.55 us times 2ALS_VIS_ADC_GAIN) */

/**************************************************************************//**
 * @brief Si1145 Commands. For every write to the Command register, the
 *           following sequence is required:
 *           1. Write 0x00 to Command register to clear the Response register.
 *           2. Read Response register and verify contents are 0x00.
 *           3. Write Command value from Table 5 into Command register.
 *           4. Read the Response register and verify contents are now non-zero.
 *               If contents are still 0x00, repeat this step. (not applicable
 *               to the Reset Command because the device will reset itself and
 *               does not increment the Response register after reset.
 *       No Commands should be issued to the device for at least 1 ms after a
 *       Reset is issued. The Response register will be incremented upon the
 *       successful completion of a Command. If the Response register remains
 *       0x00 for over 25 ms after the Command write, the entire Command process
 *       should be repeated starting with Step 1.
 ******************************************************************************/
#define SI1145_CMD_PARAM_QUERY(param)   (0x80 | param)  /**< Reads the parameter pointed to by bitfield [4:0] and writes value to PARAM_RD. */
#define SI1145_CMD_PARAM_SET(param)     (0xA0 | param)  /**< Sets parameter pointed by bitfield [4:0] with value in PARAM_WR, and writes value out to PARAM_RD. */
#define SI1145_CMD_NOP                  (0x00)          /**< 0b 0000 0000; Forces a zero into the RESPONSE register */
#define SI1145_CMD_RESET                (0x01)          /**< 0b 0000 0001; Performs a software reset of the firmware */
#define SI1145_CMD_BUSADDR              (0x02)          /**< 0b 0000 0010; Modifies I2C address */
#define SI1145_CMD_RESERVED1            (0x03)          /**< 0b 0000 0011; Reserved */
#define SI1145_CMD_RESERVED2            (0x04)          /**< 0b 0000 0100; Reserved */
#define SI1145_CMD_PS_FORCE             (0x05)          /**< 0b 0000 0101; Forces a single PS measurement */
#define SI1145_CMD_ALS_FORCE            (0x06)          /**< 0b 0000 0110; Forces a single ALS measurement */
#define SI1145_CMD_PSALS_FORCE          (0x07)          /**< 0b 0000 0111; Forces a single PS and ALS measurement */
#define SI1145_CMD_RESERVED3            (0x08)          /**< 0b 0000 1000; Reserved */
#define SI1145_CMD_PS_PAUSE             (0x09)          /**< 0b 0000 1001; Pauses autonomous PS */
#define SI1145_CMD_ALS_PAUSE            (0x0A)          /**< 0b 0000 1010; Pauses autonomous ALS */
#define SI1145_CMD_PSALS_PAUSE          (0x0B)          /**< 0b 0000 1011; Pauses PS and ALS */
#define SI1145_CMD_RESERVED4            (0x0C)          /**< 0b 0000 1100; Reserved */
#define SI1145_CMD_PS_AUTO              (0x0D)          /**< 0b 0000 1101; Starts/Restarts an autonomous PS Loop */
#define SI1145_CMD_ALS_AUTO             (0x0E)          /**< 0b 0000 1110; Starts/Restarts an autonomous ALS Loop */
#define SI1145_CMD_PSALS_AUTO           (0x0F)          /**< 0b 0000 1111; Starts/Restarts autonomous ALS and PS loop */
#define SI1145_CMD_GET_CAL              (0x12)          /**< 0b 0001 0010; Reports calibration data to I2C registers 0x22-0x2D */

/**************************************************************************//**
 * @brief Si1145 error responses
 ******************************************************************************/
#define SI1145_ERRRSP_NO_ERROR_0            (0x00)  /**< 0b 0000 0000; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_1            (0x01)  /**< 0b 0000 0001; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_2            (0x02)  /**< 0b 0000 0010; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_3            (0x03)  /**< 0b 0000 0011; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_4            (0x04)  /**< 0b 0000 0100; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_5            (0x05)  /**< 0b 0000 0101; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_6            (0x06)  /**< 0b 0000 0110; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_7            (0x07)  /**< 0b 0000 0111; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_8            (0x08)  /**< 0b 0000 1000; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_9            (0x09)  /**< 0b 0000 1001; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_A            (0x0A)  /**< 0b 0000 1010; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_B            (0x0B)  /**< 0b 0000 1011; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_C            (0x0C)  /**< 0b 0000 1100; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_D            (0x0D)  /**< 0b 0000 1101; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_E            (0x0E)  /**< 0b 0000 1110; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_NO_ERROR_F            (0x0F)  /**< 0b 0000 1111; The lower bit is a circular counter and is incremented every time a command has completed. This allows the host to keep track of commands sent to the Si1145/46/47. The circular counter may be cleared using the NOP command. */
#define SI1145_ERRRSP_INVALID_SETTING       (0x80)  /**< 0b 1000 0000; An invalid setting was encountered. Clear using the NOP command. */
#define SI1145_ERRRSP_PS1_ADC_OVERFLOW      (0x88)  /**< 0b 1000 1000; Indicates proximity channel one conversion overflow. */
#define SI1145_ERRRSP_PS2_ADC_OVERFLOW      (0x89)  /**< 0b 1000 1001; Indicates proximity channel two conversion overflow. */
#define SI1145_ERRRSP_PS3_ADC_OVERFLOW      (0x8A)  /**< 0b 1000 1010; Indicates proximity channel three conversion overflow. */
#define SI1145_ERRRSP_ALS_VIS_ADC_OVERFLOW  (0x8C)  /**< 0b 1000 1100; Indicates visible ambient light channel conversion overflow. */
#define SI1145_ERRRSP_ALS_IR_ADC_OVERFLOW   (0x8D)  /**< 0b 1000 1101; Indicates infrared ambient light channel conversion overflow. */
#define SI1145_ERRRSP_AUX_ADC_OVERFLOW      (0x8E)  /**< 0b 1000 1110; Indicates auxiliary channel conversion overflow. */

/**
 * @brief   Initializes the Si1145 Light Sensor
 * @param[in] i2cspm
 *   The I2C peripheral to use.
 *
 * @return SL_STATUS_OK on Success; SL_STATUS_INITIALIZATION on Failure
 */
sl_status_t
si1145_init(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 * @brief
 *    Stop the measurements on all channel and waits until the chip
 *    goes to sleep state.
 *
 * @param[in] i2cspm
 *   The I2C peripheral to use.
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_TRANSMIT I2C transmit failure
 ******************************************************************************/
sl_status_t
si1145_deinit(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 * @brief
 *    Measure lux and UV index using the Si1145 sensor.
 *
 * @param[in] i2cspm
 *   The I2C peripheral to use.
 *
 * @param[out] lux
 *    The measured ambient light illuminance in lux
 *
 * @param[out] uvi
 *    The measured UV index
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_TRANSMIT I2C transmit failure
 ******************************************************************************/
sl_status_t
si1145_measure_lux_uvi(sl_i2cspm_t *i2cspm, float *lux, float *uvi);

/***************************************************************************//**
 * @brief
 *    Read register from the Si1145 sensor.
 *
 * @param[in] i2cspm
 *   The I2C peripheral to use.
 *
 * @param[in] reg
 *    The register address to read from in the sensor.
 *
 * @param[out] data
 *    The data read from the sensor
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_TRANSMIT I2C transmit failure
 ******************************************************************************/
sl_status_t
si1145_read_register(sl_i2cspm_t *i2cspm, uint8_t reg, uint8_t *data);

/***************************************************************************//**
 * @brief
 *    Write register in the Si1145 sensor.
 *
 * @param[in] i2cspm
 *   The I2C peripheral to use.
 *
 * @param[in] reg
 *    The register address to write to in the sensor
 *
 * @param[in] data
 *    The data to write to the sensor
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_TRANSMIT I2C transmit failure
 ******************************************************************************/
sl_status_t
si1145_write_register(sl_i2cspm_t *i2cspm, uint8_t reg, uint8_t data);

/***************************************************************************//**
 * @brief
 *    Write a block of data to the Si1145 sensor.
 *
 * @param[in] i2cspm
 *   The I2C peripheral to use.
 *
 * @param[in] reg
 *    The first register to begin writing to
 *
 * @param[in] length
 *    The number of bytes to write to the sensor
 *
 * @param[in] data
 *    The data to write to the sensor
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_TRANSMIT I2C transmit failure
 ******************************************************************************/
sl_status_t
si1145_write_register_block(sl_i2cspm_t *i2cspm, uint8_t reg, uint8_t length,
		const uint8_t *data);

/***************************************************************************//**
 * @brief
 *    Read a block of data from the Si1145 sensor.
 *
 * @param[in] i2cspm
 *   The I2C peripheral to use.
 *
 * @param[in] reg
 *    The first register to begin reading from
 *
 * @param[in] length
 *    The number of bytes to write to the sensor
 *
 * @param[out] data
 *    The data read from the sensor
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_TRANSMIT I2C transmit failure
 ******************************************************************************/
sl_status_t
si1145_read_register_block(sl_i2cspm_t *i2cspm, uint8_t reg, uint8_t length,
		uint8_t *data);

/***************************************************************************//**
 * @brief
 *    Reset the Si1145.
 *
 * @param[in] i2cspm
 *   The I2C peripheral to use.
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_TRANSMIT I2C transmit failure
 ******************************************************************************/
sl_status_t
si1145_reset(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 * @brief
 *    Send a PAUSE command to the Si1145.
 *
 * @param[in] i2cspm
 *   The I2C peripheral to use.
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_TRANSMIT I2C transmit failure
 ******************************************************************************/
sl_status_t
si1145_pause_measurement(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 * @brief
 *    Write a byte to an Si1145 Parameter.
 *
 * @param[in] i2cspm
 *   The I2C peripheral to use.
 *
 * @param[in] address
 *    The parameter address
 *
 * @param[in] value
 *    The byte value to be written to the Si1145 parameter
 *
 * @retval SL_STATUS_OK Success
 * @retval SL_STATUS_TRANSMIT I2C transmit failure
 *
 * @note
 *    This function ensures that the Si1145 is idle and ready to
 *    receive a command before writing the parameter. Furthermore,
 *    command completion is checked. If setting parameter is not done
 *    properly, no measurements will occur. This is the most common
 *    error. It is highly recommended that host code make use of this
 *    function.
 ******************************************************************************/
sl_status_t
si1145_set_parameter(sl_i2cspm_t *i2cspm, uint8_t address, uint8_t value);

#ifdef __cplusplus
}
#endif
#endif /* DRIVERS_SI1145_H_ */
