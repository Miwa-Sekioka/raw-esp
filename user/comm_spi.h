#ifdef  _COMM_SPI_H_
#define _COMM_SPI_H_

#include <stdint.h>
#include <stdbool.h>

//SPI Initialize
void spi_init();

//Tx Rx API
void spi_send_data(uint8_t *data, uint32_t length);
void spi_get_recv_data(uint8_t **data, uint32_t &length);
void spi_recv_buffer_clear();

//check tx buffer
bool spi_send_ready();

//handler function
void spi_recv_handler(uint8_t *data, uint32_t length);
void spi_send_end_handler();

#endif

