#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"


// SHT31's SCL AND SDA PINS on nRF"
#define DEVICE_SCL_PIN 29
#define DEVICE_SDA_PIN 30
#define SLAVE_ADDRESS 0x44


void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

uint32_t twi_init (nrf_drv_twi_t * p_twi);

uint32_t get_temp_humid(nrf_drv_twi_t * p_twi);
