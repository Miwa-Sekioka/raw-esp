#ifndef _ESP_SPI_H_
#define _ESP_SPI_H_

#include <stdint.h>
#include <stdbool.h>

#define ESP_BUFFER_FULL_MASK  0x01
#define ESP_DATA_MASK   0x02


bool spi_init();
void spi_close();

bool spi_read_st(uint8_t *st);

bool spi_recv_data(uint8_t **data, uint32_t *length);
bool spi_send_data(uint8_t *data , uint32_t  length);

#endif
