#include <stdio.h>

#include "utility.h"

#ifdef PRINT_RAW

void print_hex(uint8_t *data, uint32_t length)
{
  while(length-- > 0){
    printf("%02x", *(data++));
  }
}

#endif

uint32_t ipstr_to_uint32(uint8_t *ipstr)
{
  int a1,a2,a3,a4;
  int ret = sscanf(ipstr, "%d.%d.%d.%d", &a1, &a2, &a3, &a4);
  if(ret == 4)
  {
    return ip_to_uint32(a1, a2, a3, a4);
  }else{
    return 0;
  }
}

uint32_t ip_to_uint32(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4)
{
  return a1<<24|a2<<16|a3<<8|a4;
}

