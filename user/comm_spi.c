/*
 * comm_spi.c
 * host interface via spi0
 *
 * host communication packet(Each packet is 32byte const length)
 *
 * Head Packet: F0 00 LLLL(Data length:little dndiation) D0 D1 D2....D27
 * N th packet: F0 II(Packet index) D28 D29...
 */

#include "driver/spi_interface.h"
#include "eagle_soc.h"
#include "osapi.h"

#include "user_interface.h"
#include "comm_spi.h"

#define BUFFER_MAX 1600
#define HEAD_MTU 28
#define MTU 30


//WR = b0=RECV_BUFFER(1:FULL,0:EMPTY), b1:SEND_BUFFER(1:FULL, 0:EMPTY)
#define ESP_BUFFER_FULL_MASK  0x01
#define ESP_DATA_MASK   0x02

static void _on_read_end();
static void send_data(uint8_t *data, uint32_t length);

uint8_t recv_buffer[BUFFER_MAX];
uint16_t recv_length;
uint16_t recv_index;

uint8_t send_buffer[BUFFER_MAX];
uint16_t send_length;
uint16_t send_index;

static void set_wr_status(uint8_t mask, uint8_t value)
{
    uint8_t status = READ_PERI_REG(SPI_WR_STATUS(SpiNum_HSPI));
    status = (status & ~mask) | value;
    WRITE_PERI_REG(SPI_WR_STATUS(SpiNum_HSPI), status);
}

static void _on_write()
{
  uint8_t buffer[32];
  
  uint32_t i;
  for (i = 0; i < 8; i++) {
    ((uint32_t*)buffer)[i] = READ_PERI_REG(SPI_W0(SpiNum_HSPI) + (i<<2));
  }
  
  if(buffer[0] == 0xf0){
    if(buffer[1] == 0x00){
      if(*(uint16_t*)&buffer[2] <= BUFFER_MAX){
        recv_length = *(uint16_t*)&buffer[2];
        os_memcpy(recv_buffer, &buffer[4], HEAD_MTU);
        recv_index = 1;
      }else{
        recv_length = 0;
      }
    }else if(buffer[1] == recv_index){
      os_memcpy(&recv_buffer[HEAD_MTU+MTU*(recv_index-1)], &buffer[2], MTU);
      recv_index++;
    }
    
    if(recv_index > 0){
      if(HEAD_MTU + MTU*(recv_index-1) >= recv_length){
        //RECV BUFFER FULL
        set_wr_status(0x01, 0x01);
        spi_recv_handler(recv_buffer, recv_length);
      }    
    }
  }
}

bool spi_send_ready()
{
  uint8_t status = READ_PERI_REG(SPI_WR_STATUS(SpiNum_HSPI));
  return (status & ESP_DATA_MASK) == 0;
}

void spi_recv_buffer_clear()
{
  recv_index=0;
  //RECV BUFFER CLEAR
  set_wr_status(0x01, 0x00);
}

void spi_get_recv_data(uint8_t **data, uint32_t *length)
{
  if(recv_length != 0){
    *data = recv_buffer;
    *length = recv_length;
  }else{
    *data = NULL;
  }
}

static void _send_data()
{
  uint8_t buffer[MTU+2];//MTU(30)+2=32
    
  if(send_index == 0)
  {
    uint32_t remain = send_length;
    if(remain > HEAD_MTU){
      remain = HEAD_MTU;
    }
    buffer[0] = 0xF0;
    buffer[1] = 0x00;
    *(uint16_t*)&buffer[2] = send_length;
    os_memcpy(&buffer[4], &send_buffer[0], remain);
  }else{
    uint32_t remain = send_length - HEAD_MTU - MTU*(send_index-1);
    if(remain > MTU){
      remain = MTU;
    }
    buffer[0] = 0xF0;
    buffer[1] = send_index;
    os_memcpy(&buffer[2], &send_buffer[HEAD_MTU+MTU*(send_index-1)], remain); 
  }
  
  SPISlaveSendData(SpiNum_HSPI, (uint32_t*)buffer, sizeof(buffer));
  
  if(send_index == 0){
    //SEND BUFFER SET
    set_wr_status(0x02, 0x02); 
  }
  
  send_index++;
}

void spi_send_data(uint8_t *data, uint32_t length)
{
  if(length > sizeof(send_buffer)){
    return;
  }
  os_memcpy(send_buffer, data, length);
  send_length = length;
  send_index = 0;
  _send_data();
}

static void _on_read_end()
{
  if(HEAD_MTU + MTU*(send_index-1) < send_length)
  {
    _send_data();
  }else{
    //ReadEnd
    set_wr_status(0x02, 0x00);
    spi_send_end_handler(0x00);
  }
}

// SPI interrupt callback function.
static void _spi_slave_isr_sta(void *para)
{
  uint32 regvalue;
  uint32 statusW, statusR, counter;
  if (READ_PERI_REG(0x3ff00020)&BIT7) { //bit7 is for hspi isr,
    regvalue = READ_PERI_REG(SPI_SLAVE(SpiNum_HSPI));
    //os_printf("spi_slave_isr_sta SPI_SLAVE[0x%08x]\n\r", regvalue);
    
    SPIIntDisable(SpiNum_HSPI, SpiIntSrc_WrBufDone | SpiIntSrc_RdBufDone);
    SPIIntClear(SpiNum_HSPI);
    SET_PERI_REG_MASK(SPI_SLAVE(SpiNum_HSPI), SPI_SYNC_RESET);
    SPIIntClear(SpiNum_HSPI);
    
    if (regvalue & SPI_SLV_WR_BUF_DONE) {
      //os_printf("spi_slave_isr_sta SPI_SLV_WR_BUF_DONE\n\r");
      _on_write();
    } else if (regvalue & SPI_SLV_RD_BUF_DONE) {
      //os_printf("spi_slave_isr_sta : SPI_SLV_RD_BUF_DONE\n\r");            
      _on_read_end();
    }
    
    SPIIntEnable(SpiNum_HSPI, SpiIntSrc_WrBufDone | SpiIntSrc_RdBufDone);
  }else if (READ_PERI_REG(0x3ff00020)&BIT4) {
    //following 3 lines is to clear isr signal
    CLEAR_PERI_REG_MASK(SPI_SLAVE(SpiNum_SPI), 0x3ff);
  }
}

void spi_init()
{
  SpiAttr hSpiAttr;
  hSpiAttr.bitOrder = SpiBitOrder_MSBFirst;
  hSpiAttr.speed = 0;
  hSpiAttr.mode = SpiMode_Slave;
  hSpiAttr.subMode = SpiSubMode_0;

  // Init HSPI GPIO
  WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2);//configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2);//configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2);//configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2);//configure io to spi mode

  SPIInit(SpiNum_HSPI, &hSpiAttr);

  recv_index = 0;
  send_index = 0;
  WRITE_PERI_REG(SPI_RD_STATUS(SpiNum_HSPI), 0x00);
  WRITE_PERI_REG(SPI_WR_STATUS(SpiNum_HSPI), 0x00);
  
  // Set spi interrupt information.
  SpiIntInfo spiInt;
  spiInt.src = (SpiIntSrc_TransDone 
      |SpiIntSrc_WrBufDone 
      |SpiIntSrc_RdBufDone);
      
  spiInt.isrFunc = _spi_slave_isr_sta;
  SPIIntCfg(SpiNum_HSPI, &spiInt);
  SPISlaveRecvData(SpiNum_HSPI);
}

