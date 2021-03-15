// MASTER
// MCP4441 Digital Potentiometer
// Georgia Lawlor
// 02/05/2021

#include <__cross_studio_io.h>
#include <stdio.h>
#include <nrfx.h>
#include "nrfx_twi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "i2c_header.h"

/* TWI instance. */
static const nrfx_twi_t twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

// TWI initialization.
void twi_init (void)
{
    ret_code_t err_code;

    const nrfx_twi_config_t twi_config = {
       .scl                = TWI_SCL,
       .sda                = TWI_SDA,
       .frequency          = NRFX_TWI_DEFAULT_CONFIG_FREQUENCY,
       .interrupt_priority = NRFX_TWI_DEFAULT_CONFIG_IRQ_PRIORITY,
       .hold_bus_init      = NRFX_TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT
    };

    err_code = nrfx_twi_init(&twi, &twi_config, NULL, NULL);
    printf("error code: %s \n", err_code)

    nrfx_twi_enable(&twi);
}

// void MCP4441_set_mode(void) {
//     ret_code_t err_code;

//     /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
//     uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
//     err_code = nrfx_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
//     APP_ERROR_CHECK(err_code);
//     while(m_xfer_done == false);

//     /* Writing to pointer byte. */
//     reg[0] = LM75B_REG_TEMP;
//     m_xfer_done = false;
//     err_code = nrfx_twi_tx(&m_twi, LM75B_ADDR, reg, 1, false);
//     APP_ERROR_CHECK(err_code);
//     while(m_xfer_done == false);
// }

// Function for handling data from temperature sensor.
__STATIC_INLINE void data_handler(uint8_t temp) {
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

// \\ TWI events handler.
// void twi_handler(nrfx_twi_evt_t const * p_event, void * p_context) {
//     switch (p_event->type)
//     {
//         case NRFX_TWI_EVT_DONE:
//             if (p_event->xfer_desc.type == NRFX_TWI_XFER_RX)
//             {
//                 data_handler(m_sample);
//             }
//             m_xfer_done = true;
//             break;
//         default:
//             break;
//     }
// }

// Function for reading data from temperature sensor.
static void read_status_data() {
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code 
    #define NRFX_TWI_XFER_DESC_TXRX(MCP4441_ADDR, *STATUS, 1, *, 2)
        { 
            .type             = (nrfx_twim_xfer_type_t)p_xfer_desc->type,
            .address          = p_xfer_desc->address,
            .primary_length   = p_xfer_desc->primary_length,
            .secondary_length = p_xfer_desc->secondary_length,
            .p_primary_buf    = p_xfer_desc->p_primary_buf,
            .p_secondary_buf  = p_xfer_desc->p_secondary_buf,
        };
        result = nrfx_twim_xfer(&p_instance->u.twim, &twim_xfer_desc, flags);
    NRFX_TWI_XFER_DESC_TXRT(MCP4441_ADDR, 1, 2, pointer to write, pointer to read)
    err_code = nrfx_twi_xfer(&m_twi, NRFX_TWI_XFER_TXRX, MCP4441_ADDR, &m_sample, 1);
    printf("error code: %s \n", err_code)
    err_code = nrfx_twi_rx(&m_twi, MCP4441_ADDR, &m_sample, 1);
}

// Function for main application entry.
int main(void) {
    twi_init();
    read_status_data();

    // while (true) {
    //     nrf_delay_ms(500);
    //     do {
    //         __WFE();
    //     } while (m_xfer_done == false);
    //     read_sensor_data();
    // }
}