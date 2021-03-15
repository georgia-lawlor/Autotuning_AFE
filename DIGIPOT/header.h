/**
 * MASTER
 * MCP4441 Digital Potentiometer
 * Georgia Lawlor
 * 02/19/2021
 */
//#include <nrf.h>

#ifndef HEADER_H
#define HEADER_H
#endif

/* TWI Master Initialization */
#define BIONODE_I2C_SDA  26  // HW SDA
#define BIONODE_I2C_SCL  27  // HW SCL

/* Pin Initializations */
// ADC Input Channels
#define DP_WPR0_AIN    1           // ADC Input for Wiper 0
// GPIOs
#define DP_WP_LOCK    17           // GPIO Wiper Lock
#define DP_RST        18           // GPIO Reset
#define DP_A1         19           // GPIO A1 - HW Address
#define DP_HVC_A0     20           // GPIO A0 & High Voltage Control - HW Address
#define DP_SUCCESS    22 

/* Device Address */
#define DP_ADDR       0b0101100  // Base Address, Add A1 & A0

/* Registers */
#define DP_VWIPER0    0x00     // Volatile Wiper 0
#define DP_VWIPER1    0x01     // Volatile Wiper 1
#define DP_NVWIPER0   0x02     // NonVolatile Wiper 0
#define DP_NVWIPER1   0x03     // NonVolatile Wiper 1
#define DP_VTCON0     0x04     // Volatile TCON 0
#define DP_STATUS     0x05     // Status Register

/* Commands */
#define DP_WRITE_CMD  0b00     // Write
#define DP_INCR_CMD   0b01     // Increment
#define DP_DECR_CMD   0b10     // Decrement
#define DP_READ_CMD   0b11     // Read

/* R/W bit */
#define DP_WRITE_BIT  0b00
#define DP_READ_BIT   0b01