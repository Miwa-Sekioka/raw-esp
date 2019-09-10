#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#include "esp_spi.h"
#include "utility.h"


#define BUFFER_LENGTH 2048
#define SPI_SPEED 10000000
#define SPI_CS 8
#define HEAD_MTU 28
#define MTU 30

#define ESP_SPI_MTU 34
#define ESP_SPI_WR 0x02
#define ESP_SPI_RD 0x03
#define ESP_SPI_RD_ST 0x04
#define ESP_SPI_ADDRESS 0x00
#define ESP_HEAD_LENGTH 2
#define ESP_BUFFER_FULL_MASK  0x01
#define ESP_DATA_MASK   0x02
#define WAIT_SPI 50

static uint8_t  send_buffer[BUFFER_LENGTH];
static uint32_t send_length;
static uint8_t  recv_buffer[BUFFER_LENGTH];
static uint32_t recv_index;
static uint32_t recv_length;
static int fd;

static bool pi_send(int fd, uint8_t *data, uint32_t length)
{
  int ret;
  uint8_t rx[length];
  struct spi_ioc_transfer tr;
  memset(&tr, 0x00, sizeof(tr));
  tr.tx_buf = (unsigned long)data;
  tr.rx_buf = (unsigned long)rx;
  tr.len = length;
  tr.delay_usecs = 0;
  tr.speed_hz = SPI_SPEED;
  tr.bits_per_word = 8;
  tr.cs_change = 1;
  
#ifdef PRINT_RAW_DATA
  printf(">>:");
  print_hex(data, length);
  printf("\n");
#endif
  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  if(ret < 0){
    printf("pi_send error\n");
  }
    
  usleep(WAIT_SPI);
  return ret >= 0;
}

static bool pi_recv(int fd, uint8_t *data, uint32_t length)
{
  int ret;
  uint8_t read_cmd[length];
  read_cmd[0] = ESP_SPI_RD;
  read_cmd[1] = ESP_SPI_ADDRESS;
  struct spi_ioc_transfer tr;
  memset(&tr, 0x00, sizeof(tr));
  tr.tx_buf = (unsigned long)read_cmd;
  tr.rx_buf = (unsigned long)data;
  tr.len = length;
  tr.delay_usecs = 0;
  tr.speed_hz = SPI_SPEED;
  tr.bits_per_word = 8;
  tr.cs_change = 1;

  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  if(ret < 0){
    printf("pi_read error\n");
  }else{

#ifdef PRINT_RAW_DATA
    printf("<<:");
    print_hex(data, ret);
    printf("\n");
#endif
  }
  
  usleep(WAIT_SPI);
  return ret == length;
}

static bool pi_read_st(int fd, uint8_t *data)
{
  int ret;
  static uint8_t count = 0;
  uint8_t read_cmd[2];
  uint8_t buffer[2];
  
  read_cmd[0] = ESP_SPI_RD_ST;
  read_cmd[1] = ESP_SPI_ADDRESS;
  
  struct spi_ioc_transfer tr;  
  memset(&tr, 0x00, sizeof(tr));
  tr.tx_buf = (unsigned long)read_cmd;
  tr.rx_buf = (unsigned long)buffer;
  tr.len = 2;
  tr.delay_usecs = 0;
  tr.speed_hz = SPI_SPEED;
  tr.bits_per_word = 8;
  tr.cs_change = 1;

  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  if(ret < 0){
    printf("pi_rad_st error\n");
  }
  *data = buffer[1];
  
  //動作中確認（フリーズしていない事の確認用）
  /*
  if(*data == 0x00){
    printf(".");
    if(count++ % 200 == 0){
      printf("\n");
    }
  }*/
  return ret == 2;
}

bool spi_send_data(uint8_t *data, uint32_t length)
{
  int i;
  memcpy(send_buffer, data, length);
  send_length = length;
  i=0;
  do{
    uint8_t buffer[ESP_SPI_MTU];
    
    buffer[0] = ESP_SPI_WR;
    buffer[1] = ESP_SPI_ADDRESS;
    if(i == 0){
      uint32_t remain = send_length;
      if(remain > HEAD_MTU){
        remain = HEAD_MTU;
      }
      buffer[2] = 0xF0;
      buffer[3] = i;
      *(uint16_t*)&buffer[4] = length;
      memcpy(&buffer[6], &send_buffer[0], remain); 
      pi_send(fd, buffer, sizeof(buffer));
    }else{
      uint32_t remain = send_length - HEAD_MTU - MTU*(i-1);
      if(remain > MTU){
        remain = MTU;
      }
      buffer[2] = 0xF0;
      buffer[3] = i;
      memcpy(&buffer[4], &send_buffer[HEAD_MTU+MTU*(i-1)], remain); 
      if(!pi_send(fd, buffer, sizeof(buffer))){
        return false;
      }
    }
    i++;
  }while(HEAD_MTU + MTU*(i-1)<send_length);
  
  return true;
}

//先頭まですすめる
static void _peek_head()
{
  while(1)
  {
    uint8_t buffer[ESP_SPI_MTU];
    uint8_t st;
    if(!pi_read_st(fd, &st)){
      break;
    }
    
    if(st == 0){
      break;
    }
    
    if(!pi_recv(fd, buffer, sizeof(buffer))){
      break;
    }
  }
}

bool spi_recv_data(uint8_t **data, uint32_t *length)
{
  recv_index = 0;
  while(1)
  {
    uint8_t buffer[ESP_SPI_MTU];
    if(!pi_recv(fd, buffer, sizeof(buffer))){
      return false;
    }
    
    if(buffer[ESP_HEAD_LENGTH] == 0xf0)
    {
      if(buffer[ESP_HEAD_LENGTH+1] == 0x00){
        if(*(uint16_t*)&buffer[ESP_HEAD_LENGTH+2] <= BUFFER_LENGTH)
        {
          recv_length = *(uint16_t*)&buffer[ESP_HEAD_LENGTH+2];
          memcpy(recv_buffer, &buffer[ESP_HEAD_LENGTH+4],  sizeof(buffer) - ESP_HEAD_LENGTH);
          recv_index=1;
        }else{
          printf("recv buffer over\n");
          //残りパケットを受信
          _peek_head();
          break;
        }
      }else{
        if(recv_index == buffer[ESP_HEAD_LENGTH+1]){
          memcpy(&recv_buffer[HEAD_MTU + MTU*(recv_index-1)], &buffer[ESP_HEAD_LENGTH+2],  sizeof(buffer) - ESP_HEAD_LENGTH);
          recv_index++;
        }else{
          printf("recv packet loss\n");
          //残りパケットを受信
          _peek_head();
          break;
        }
      }
      
      //recv complete
      if(recv_length <= HEAD_MTU + MTU*(recv_index-1)){
        *data = recv_buffer;
        *length = recv_length;
        return true;
      }
    }else{
      break;
    }
  }
  return false;
}

bool spi_init(uint8_t *device)
{
  unsigned char mode = 0;
  unsigned char bits = 8;
  unsigned long speed = SPI_SPEED;
  uint16_t delay;
  int ret = 0;
  int i=0;
  
  printf("open %s\n", device);
  fd = open(device, O_RDWR);
  if(fd == -1){
    printf("can't open device\n"); 
    return false;
  }
   
  ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
  if(ret == -1){
    printf("can't set spi mode");
    return false;
  }

  if(ioctl(fd, SPI_IOC_RD_MODE, &mode) == -1){
    printf("can't set spi mode");
    return false;
  }

  if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1){
    printf("can't set spi mode");
    return false;
  }

  if(ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1){
    printf("can't set spi mode");
    return false;
  }

  if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1){
    printf("can't set spi mode");
    return false;
  }

  if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1){
    printf("can't set spi mode");
    return false;
  }
  
  printf("SPI initialize complete!\n");
  
  return true;
}

void spi_close()
{
  if(fd > 0){
    close(fd);
    fd = 0;
  }
}

bool spi_read_st(uint8_t *st){
  if(!pi_read_st(fd, st)){
    return false;
  }
  return true;
}
