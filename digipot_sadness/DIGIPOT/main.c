/**
 * MASTER
 * MCP4441 Digital Potentiometer
 * Georgia Lawlor
 * 02/19/2021
 */
#include <__cross_studio_io.h>
#include <stdio.h>
#include <nrf.h>

#include "header.h"
#include "twi_master.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

/* Function Declarations */
int ADC_Init(void);
int GPIO_Init(void);

/* Global Variables */
bool initializer;
bool successful;

// Function for main application entry.
int main(void) {
    // Initialize Peripherals
    GPIO_Init();
    initializer = twi_master_init();

    // Read from status register
    uint8_t reg_cmd[1] = {((DP_STATUS << 4) | (DP_READ_CMD << 2))};
    uint8_t status_reg[2];
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd, sizeof(reg_cmd), false);  
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_READ_BIT), status_reg, sizeof(status_reg), true);
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    else {
        NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
    }
    
    // Increment VWIPER 0
    uint8_t reg_cmd1[1] = {((DP_VWIPER0 << 4) | (DP_DECR_CMD << 2))};
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd1, sizeof(reg_cmd1), true);  
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    else {
        NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
    }

    // Read from VWIPER 0
    uint8_t DP_VWIPER0_reg[2];
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_READ_BIT), DP_VWIPER0_reg, sizeof(DP_VWIPER0_reg), true);
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    else {
        NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
    }
    
    // Write to NVWIPER 0
    uint8_t reg_cmd2[2] = {((DP_NVWIPER0 << 4) | (DP_WRITE_CMD << 2)), (DP_VWIPER0_reg[1])};
    successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd2, sizeof(reg_cmd2), true);
    if (successful) {
        NRF_GPIO->OUTSET = (1 << DP_SUCCESS); // set high
    }
    else {
        NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low
    }

    //uint8_t reg_cmd12[1] = {((DP_VWIPER0 << 4) | (DP_INCR_CMD << 2))};
    //successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd12, sizeof(reg_cmd12), true);  
    //for (int i = 0; i < 500; i++) {}
    //uint8_t reg_cmd13[1] = {((DP_VWIPER0 << 4) | (DP_INCR_CMD << 2))};
    //successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd13, sizeof(reg_cmd13), true);  
    //for (int i = 0; i < 500; i++) {}
    //uint8_t reg_cmd14[1] = {((DP_VWIPER0 << 4) | (DP_INCR_CMD << 2))};
    //successful = twi_master_transfer(((DP_ADDR << 1) | DP_WRITE_BIT), reg_cmd14, sizeof(reg_cmd14), true);  
}

// Function to initialize and set all GPIOs
int GPIO_Init(void)
{
  //Set the Wiper Lock GPIO to be an output pin
  NRF_GPIO->PIN_CNF[DP_WP_LOCK] = GPIO_PIN_CNF_DIR_Output;
  NRF_GPIO->DIRSET |= 1 << DP_WP_LOCK;
  NRF_GPIO->OUTSET = (1 << DP_WP_LOCK); // set high

  //Set up the Reset GPIO to be an output pin
  NRF_GPIO->PIN_CNF[DP_RST] = GPIO_PIN_CNF_DIR_Output;
  NRF_GPIO->DIRSET |= 1 << DP_RST;
  NRF_GPIO->OUTSET = (1 << DP_RST); // set high

  //Set up the A1 GPIO to be an output pin
  NRF_GPIO->PIN_CNF[DP_A1] = GPIO_PIN_CNF_DIR_Output;
  NRF_GPIO->DIRSET |= 1 << DP_A1;
  NRF_GPIO->OUTCLR = (1 << DP_A1); // set low

  //Set up the High Voltage Control/A0 GPIO to be an output
  NRF_GPIO->PIN_CNF[DP_HVC_A0] = GPIO_PIN_CNF_DIR_Output;
  NRF_GPIO->DIRSET |= 1 << DP_HVC_A0;
  NRF_GPIO->OUTCLR = (1 << DP_HVC_A0); // set low

  //Set up the Transfer Success? GPIO to be an output
  NRF_GPIO->PIN_CNF[DP_SUCCESS] = GPIO_PIN_CNF_DIR_Output;
  NRF_GPIO->DIRSET |= 1 << DP_SUCCESS; 
  NRF_GPIO->OUTCLR = (1 << DP_SUCCESS); // set low

  return 0;
}
