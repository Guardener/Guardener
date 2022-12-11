#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[ACMP0]
// [ACMP0]$

// $[ACMP1]
// [ACMP1]$

// $[ADC0]
// [ADC0]$

// $[CMU]
// [CMU]$

// $[DBG]
// DBG SWV on PF2
#define DBG_SWV_PORT                             gpioPortF
#define DBG_SWV_PIN                              2
#define DBG_SWV_LOC                              0

// [DBG]$

// $[ETM]
// [ETM]$

// $[PTI]
// PTI DFRAME on PB13
#define PTI_DFRAME_PORT                          gpioPortB
#define PTI_DFRAME_PIN                           13
#define PTI_DFRAME_LOC                           6

// PTI DOUT on PB12
#define PTI_DOUT_PORT                            gpioPortB
#define PTI_DOUT_PIN                             12
#define PTI_DOUT_LOC                             6

// [PTI]$

// $[GPIO]
// [GPIO]$

// $[I2C0]
// I2C0 SCL on PA0
#define I2C0_SCL_PORT                            gpioPortA
#define I2C0_SCL_PIN                             0
#define I2C0_SCL_LOC                             31

// I2C0 SDA on PA1
#define I2C0_SDA_PORT                            gpioPortA
#define I2C0_SDA_PIN                             1
#define I2C0_SDA_LOC                             1

// [I2C0]$

// $[I2C1]
// I2C1 SCL on PC10
#define I2C1_SCL_PORT                            gpioPortC
#define I2C1_SCL_PIN                             10
#define I2C1_SCL_LOC                             18

// I2C1 SDA on PC11
#define I2C1_SDA_PORT                            gpioPortC
#define I2C1_SDA_PIN                             11
#define I2C1_SDA_LOC                             20

// [I2C1]$

// $[LESENSE]
// [LESENSE]$

// $[LETIMER0]
// [LETIMER0]$

// $[LEUART0]
// [LEUART0]$

// $[LFXO]
// [LFXO]$

// $[MODEM]
// [MODEM]$

// $[PCNT0]
// [PCNT0]$

// $[PRS.CH0]
// [PRS.CH0]$

// $[PRS.CH1]
// [PRS.CH1]$

// $[PRS.CH2]
// [PRS.CH2]$

// $[PRS.CH3]
// [PRS.CH3]$

// $[PRS.CH4]
// [PRS.CH4]$

// $[PRS.CH5]
// [PRS.CH5]$

// $[PRS.CH6]
// [PRS.CH6]$

// $[PRS.CH7]
// [PRS.CH7]$

// $[PRS.CH8]
// [PRS.CH8]$

// $[PRS.CH9]
// [PRS.CH9]$

// $[PRS.CH10]
// [PRS.CH10]$

// $[PRS.CH11]
// [PRS.CH11]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// TIMER1 CC0 on PF7
#define TIMER1_CC0_PORT                          gpioPortF
#define TIMER1_CC0_PIN                           7
#define TIMER1_CC0_LOC                           31

// [TIMER1]$

// $[USART0]
// USART0 CTS on PA2
#define USART0_CTS_PORT                          gpioPortA
#define USART0_CTS_PIN                           2
#define USART0_CTS_LOC                           30

// USART0 RTS on PA3
#define USART0_RTS_PORT                          gpioPortA
#define USART0_RTS_PIN                           3
#define USART0_RTS_LOC                           30

// USART0 RX on PF3
#define USART0_RX_PORT                           gpioPortF
#define USART0_RX_PIN                            3
#define USART0_RX_LOC                            26

// USART0 TX on PF4
#define USART0_TX_PORT                           gpioPortF
#define USART0_TX_PIN                            4
#define USART0_TX_LOC                            28

// [USART0]$

// $[USART1]
// [USART1]$

// $[USART2]
// [USART2]$

// $[VDAC0]
// [VDAC0]$

// $[WTIMER0]
// [WTIMER0]$

// $[CUSTOM_PIN_NAME]
// [CUSTOM_PIN_NAME]$

#endif // PIN_CONFIG_H

