#include <stdint.h>
#include <stdbool.h>

void print_hex(uint8_t *data, uint32_t length);

uint32_t ipstr_to_uint32(uint8_t *ip);
uint32_t ip_to_uint32(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4);