// MASTER
// MCP4441 Digital Potentiometer
// Georgia Lawlor
// 02/02/2021

#include <nrfx.h>

#ifndef I2C_HEADER_H
#define I2C_HEADER_H
#endif


// TWI instance ID
#define TWI_INSTANCE_ID   0

''' Addresses '''
// Common addresses definition
#define MCP4441_ADDR    0b0101100

#define V_WIPER0        0x00
#define V_WIPER1        0x01
#define NV_WIPER0       0x02
#define NV_WIPER1       0x03
#define V_TCON0         0x04
#define STATUS          0x05

#define WRITE           0b000
#define INCR            0b01
#define DECR            0b10
#define READ            0b00

''' Pin Initializations '''
//ADC Input channels
#define WPR0_AIN    1

// GPIOs
#define HVC_A0      11
#define A1          12
#define RST         22
#define WP_LOCK     23

// SDA and SCL
#define TWI_SDA     26
#define TWI_SCL     27

''' TWI '''
// <e> NRFX_TWI_ENABLED  - nrfx_twi TWI peripheral driver
#ifndef NRFX_TWI_ENABLED
#define NRFX_TWI_ENABLED 1
#endif

// <q> NRFX_TWI0_ENABLED  - Enable TWI0 instance
#ifndef NRFX_TWI0_ENABLED
#define NRFX_TWI0_ENABLED 1
#endif

// <q> NRFX_TWI1_ENABLED  - Enable TWI1 instance
#ifndef NRFX_TWI1_ENABLED
#define NRFX_TWI1_ENABLED 1
#endif

// <o> NRFX_TWI_DEFAULT_CONFIG_FREQUENCY  - Frequency
// <26738688=> 100k
// <67108864=> 250k
// <104857600=> 400k
#ifndef NRFX_TWI_DEFAULT_CONFIG_FREQUENCY
#define NRFX_TWI_DEFAULT_CONFIG_FREQUENCY 26738688
#endif

// <q> NRFX_TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT  - Enables bus holding after uninit
#ifndef NRFX_TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT
#define NRFX_TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT 0
#endif

// <o> NRFX_TWI_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
// <0=> 0 (highest)
// <1=> 1
// <2=> 2
// <3=> 3
#ifndef NRFX_TWI_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_TWI_DEFAULT_CONFIG_IRQ_PRIORITY 3
#endif

// <e> NRFX_TWI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_TWI_CONFIG_LOG_ENABLED
#define NRFX_TWI_CONFIG_LOG_ENABLED 0
#endif

// <o> NRFX_TWI_CONFIG_LOG_LEVEL  - Default Severity level
// <0=> Off
// <1=> Error
// <2=> Warning
// <3=> Info
// <4=> Debug
#ifndef NRFX_TWI_CONFIG_LOG_LEVEL
#define NRFX_TWI_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_TWI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
// <0=> Default
// <1=> Black
// <2=> Red
// <3=> Green
// <4=> Yellow
// <5=> Blue
// <6=> Magenta
// <7=> Cyan
// <8=> White
#ifndef NRFX_TWI_CONFIG_INFO_COLOR
#define NRFX_TWI_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_TWI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
// <0=> Default
// <1=> Black
// <2=> Red
// <3=> Green
// <4=> Yellow
// <5=> Blue
// <6=> Magenta
// <7=> Cyan
// <8=> White
#ifndef NRFX_TWI_CONFIG_DEBUG_COLOR
#define NRFX_TWI_CONFIG_DEBUG_COLOR 0
#endif
