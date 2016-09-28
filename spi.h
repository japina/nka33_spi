#ifndef __MAIN__H_
#define __MAIN__H_
#include <stdio.h>
#include "peripherals/stm32f0xx_spi.h"
#include "peripherals/stm32f0xx_rcc.h"
#include "peripherals/stm32f0xx_gpio.h"
#include "delay.h"
#include "pins.h"

void SPI_Configure(uint16_t direction, uint16_t mode, uint16_t datasize, uint16_t cpol, uint16_t cpha, uint16_t nss, uint16_t baudrate_prescaler, uint16_t first_bit, uint16_t crc_poly, uint16_t fifo, uint8_t nss_pin);
void send_byte(uint8_t val, uint8_t nss_pin);
uint8_t send_and_read_byte(uint8_t cmd, uint8_t nss_pin);

#endif
