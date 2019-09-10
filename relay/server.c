#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/un.h>
#include <sys/select.h>
#include <poll.h>
#include <stdlib.h>
#include <memory.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "cobs.h"
#include "message.h"
#include "crc16.h"
#include <stdbool.h>
#include <errno.h>
#include <signal.h>
#include "esp_spi.h"

#define TUN_PATH "/tmp/server.sock"
#define ESP_PORT "/dev/ttyUSB0"
#define ESP_SSID "WROOM02"
#define ESP_PASS "TESTPASS"

#define READ_BUFFER_SIZE 2048
#define READ_TIMEOUT_US 1000
#define MAX_DATA_SIZE 1500

// Shift beginnig of the buffer so payload is aligned.
// This way message headers can be cast to structure directly.
#define BUF_ALIGN_OFFSET (__BIGGEST_ALIGNMENT__ - 1)
typedef void (*comm_callback_t)(uint8_t type, uint8_t *data, uint32_t len);

//Prototype
static void packet_from_esp(uint8_t type, uint8_t *data, uint32_t len);
static bool send_esp(uint8_t type, uint8_t *data, uint32_t len);
static bool init_esp();
static bool init_if();
static void on_msg_boot(uint8_t *data, uint32_t len);
static void on_msg_status(uint8_t *data, uint32_t len);
static void on_msg_log(uint8_t *data, uint32_t len);
static void on_msg_ip(uint8_t *data, uint32_t len);

//Data
struct decoder {
  struct cobs_decoder cobs;
  uint8_t buf[BUF_ALIGN_OFFSET + COBS_ENCODED_MAX_SIZE(MAX_MESSAGE_SIZE)];

  uint32_t proto_errors;
  uint32_t crc_errors;
  comm_callback_t cb;
};
struct decoder dec_esp;

enum esp_status{
  STATUS_RESET = 0,
  STATUS_IDLE,
};

enum boot_seq{
  BOOT_RESET,
  BOOT_LOG_LEVEL_SET,
  BOOT_FORWARDING_MODE_SET,
  BOOT_WIFI_MODE_SET,
  BOOT_SOFTAP_CONF_SET,
  BOOT_SOFTAP_NET_CONF_SET,
  
  BOOT_STATION_CONF_SET,
  BOOT_STATION_STATIC_IP_CONF_SET,
  BOOT_WIFI_SCAN_REQUEST,
  
  BOOT_END,
};

struct app_context{
  uint8_t boot_seq;
  int esp_port;
  int if_listen_port;
  int if_session_port;
  
  //TUN buffer
  uint8_t if_buffer[READ_BUFFER_SIZE];
  uint32_t if_buffer_index;
  uint32_t if_buffer_length;
  
  //last esp8266 spi status
  uint8_t esp_st;
};
struct app_context ctx;

static bool send_esp(uint8_t type, uint8_t *data, uint32_t len)
{
  uint8_t buf[BUF_ALIGN_OFFSET + COBS_ENCODED_MAX_SIZE(MAX_MESSAGE_SIZE)];
  uint8_t buf_cobs[BUF_ALIGN_OFFSET + COBS_ENCODED_MAX_SIZE(MAX_MESSAGE_SIZE)];
  uint16_t crc;
  uint32_t cobs_len;
  
  buf[0] = type;
  memcpy(buf+1, data, len);
  crc = crc16_block(buf, len+1);
  buf[len+1] = crc & 0xff;
  buf[len+2] = (crc>>8) & 0xff;
  if(len > MAX_DATA_SIZE)
  {
    printf(">>:%d size over¥n", len);
    return false;
  }
  //cobs encode
  cobs_len = cobs_encode(buf_cobs, buf, len+3);//type, data[n], crc16[2]
  
  printf(">>:%d:%d\n", type, len+3);
  
  return spi_send_data(buf_cobs, cobs_len);
}


static void packet_from_esp(uint8_t type, uint8_t *data, uint32_t len)
{
  printf("<<:%d:%d\n", type, len);
  
  switch(type){
    case MSG_IP_PACKET:
      on_msg_ip(data, len);
      break;      
    case MSG_BOOT:
      on_msg_boot(data, len);
      break;
    case MSG_STATUS:
      on_msg_status(data, len);
      break;
    case MSG_LOG:
      on_msg_log(data, len);
      break;
  }
}

static void on_msg_boot(uint8_t *data, uint32_t len)
{
  uint8_t cmd = LOG_LEVEL_ERROR;
  printf("log level=%d\n", cmd);
  ctx.boot_seq = BOOT_LOG_LEVEL_SET;
  send_esp(MSG_LOG_LEVEL_SET, &cmd, 1);
}

static void on_msg_log(uint8_t *data, uint32_t len)
{
  int i;
  printf("LOG:");
  for(i=0; i<len; i++){
    //show only valid ascii character
    if(data[i]>=' ' && data[i]<='~'){
      printf("%c", data[i]);
    }
  }
  printf("\n");
}

static void on_msg_ip(uint8_t *data, uint32_t len)
{
  if(ctx.boot_seq != BOOT_END){
    return;
  }
  
  if(ctx.if_session_port == 0)
  {
    return;
  }
  
  int res = write(ctx.if_session_port, data, len);
  if(res < 0){
    printf("IF closed\n");
    close(ctx.if_session_port);
    ctx.if_session_port = 0;
  }
}

static uint32_t ip_to_uint32(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4)
{
  return a1<<24|a2<<16|a3<<8|a4;
}

static void on_msg_status(uint8_t *data, uint32_t len)
{
  if(len < 1 || data[0] != 0x00){
    printf("STATUS ERROR\n");
    return;
  }
  
  switch(ctx.boot_seq){
    case BOOT_LOG_LEVEL_SET:{
      printf("Forwarding mode = ip\n");
      uint8_t cmd = FORWARDING_MODE_IP;//FORWARDING_MODE_ETHER,FORWARDING_MODE_IP;
      ctx.boot_seq = BOOT_FORWARDING_MODE_SET;
      send_esp(MSG_SET_FORWARDING_MODE, &cmd, sizeof(cmd));
    }break;  
    case BOOT_FORWARDING_MODE_SET:{
      printf("wifi mode = SoftAp\n");
      uint8_t cmd = 2;//SoftAP
      ctx.boot_seq = BOOT_WIFI_MODE_SET;
      send_esp(MSG_WIFI_MODE_SET, &cmd, sizeof(cmd));
    }break;
    case BOOT_WIFI_MODE_SET:{
      uint8_t ssid[] = ESP_SSID;
      uint8_t pass[] = ESP_PASS;
      struct msg_softap_conf cmd;
      
      printf("ssid=%s,pass=%s\n", ssid, pass);
      memset(&cmd, 0x00, sizeof(cmd));
      cmd.ssid_len = sizeof(ssid)-1;
      memcpy(cmd.ssid, ssid, sizeof(ssid)-1);
      cmd.password_len = sizeof(pass)-1;
      memcpy(cmd.password, pass, sizeof(pass)-1);
      cmd.channel = 1;
      cmd.auth_mode = WIFI_AUTH_WPA2_PSK;
      cmd.beacon_interval = 500;// 500 * 1/1024ms
      
      ctx.boot_seq = BOOT_SOFTAP_CONF_SET;
      send_esp(MSG_SOFTAP_CONF_SET, (uint8_t*)&cmd, sizeof(cmd));
    }break;
    case BOOT_SOFTAP_CONF_SET:{
      printf("APIP=192.168.4.1 DHCP=enable\n");
      struct msg_softap_net_conf cmd;
      memset(&cmd, 0x00, sizeof(cmd));
      cmd.address = htonl(ip_to_uint32(192, 168,   4, 1));
      cmd.netmask = htonl(ip_to_uint32(255, 255, 255, 0));
      cmd.gateway = htonl(ip_to_uint32(0, 0, 0, 0));
      cmd.enable_dhcpd = 1;
      cmd.dhcpd_offer_gateway = 1;
      cmd.dhcpd_first_ip = htonl(ip_to_uint32(192, 168, 4, 10));
      cmd.dhcpd_last_ip  = htonl(ip_to_uint32(192, 168, 4, 15));
      
      ctx.boot_seq = BOOT_SOFTAP_NET_CONF_SET;
      send_esp(MSG_SOFTAP_NET_CONF_SET, (uint8_t*)&cmd, sizeof(cmd));
    }break;
    case BOOT_SOFTAP_NET_CONF_SET:{
      ctx.boot_seq = BOOT_END;
      printf("BOOT Complete\n");
    }break;
  }
}

static inline void decoder_check_and_dispatch_cb(void *decoder, uint8_t *data, size_t len)
{
  struct decoder *dec = decoder;
  uint16_t crc_msg;
  uint16_t crc_calc;

  if (len < 3) {
    dec->proto_errors ++;
    return;
  }

  crc_calc = crc16_block(data, len - 2);
  memcpy(&crc_msg, data + len - 2, 2);
  if (crc_calc != crc_msg) {
    dec->crc_errors++;
    printf("CRC ERR\n");
    return;
  }

  if (dec->cb)
    dec->cb(data[0], data + 1, len - 3);
}

static void decoder_init(struct decoder *dec, comm_callback_t cb)
{
  cobs_decoder_init(
    &dec->cobs,
    dec->buf + BUF_ALIGN_OFFSET, sizeof(dec->buf) - BUF_ALIGN_OFFSET,
    decoder_check_and_dispatch_cb, dec);
  dec->proto_errors = 0;
  dec->crc_errors = 0;
  dec->cb = cb;
}

static void decoder_put(uint8_t *data, uint32_t len)
{
  cobs_decoder_put(&dec_esp.cobs, data, len);
}

static bool init_esp_uart()
{
  //Serial port configuration
  struct termios tio;
  int baudRate = B115200;
  
  fd_set readFds;
  struct timeval tv;
  ctx.esp_port = open(ESP_PORT, O_RDWR);
  if (ctx.esp_port < 0) {
    printf("esp port open error\n");
    return false;
  }
  printf("esp port open\n");

  tcgetattr( ctx.esp_port, &tio );
  tio.c_cflag = 0;
  tio.c_cflag += CREAD;               // Read enabled(Parity=None, StopBits=1, )
  tio.c_cflag += CLOCAL;              // no modemo control
  tio.c_cflag += CS8;                 // Data Bits:8bit

  cfsetispeed( &tio, baudRate );
  cfsetospeed( &tio, baudRate );
  
  //Raw mode set
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  tcsetattr( ctx.esp_port, TCSANOW, &tio );  //apply setting
  ioctl(ctx.esp_port, TCSETS, &tio);         //apply setting
  
  tv.tv_sec  = 0;
  tv.tv_usec = 1000;
  
  decoder_init(&dec_esp, packet_from_esp);
  
  //send firt command to ESP
  on_msg_boot(NULL, 0);
  
  return true;
}

static bool init_esp_spi()
{
  decoder_init(&dec_esp, packet_from_esp);
  
  if(!spi_init()){
    return false;
  }
  
  //send firt command to ESP
  on_msg_boot(NULL, 0);
  
  return true;
}

static bool init_if()
{
  struct sockaddr_un srvaddr;
    
  ctx.if_listen_port = socket(AF_UNIX, SOCK_STREAM, 0);
  if(ctx.if_listen_port < 0){
    printf("IF error:[%d]\n", errno);
    return false;
  }

  //delete socket file
  unlink(TUN_PATH);
  
  //bind socket
  memset(&srvaddr, 0, sizeof(struct sockaddr_un));
  srvaddr.sun_family = AF_UNIX;
  strcpy(srvaddr.sun_path, TUN_PATH);
  if(bind(ctx.if_listen_port, (struct sockaddr *)&srvaddr, sizeof(struct sockaddr_un)) < 0){
    printf("IF bind error:[%d]\n", errno);
    return false;
  }

  //start server
  if(listen(ctx.if_listen_port, 1) < 0){
    printf("IF listen error errno[%d]\n", errno);
    return false;
  }
  printf("IF listen start\n");
  return true;  
}

static void wait_if()
{
  struct pollfd fds[1] = {0,};
  struct sockaddr_un cliaddr;
  
  fds[0].fd = ctx.if_listen_port;
  fds[0].events = POLLIN;
  poll(fds, 1, 1);
  
  if(fds[0].revents & POLLIN){
    memset(&cliaddr, 0, sizeof(struct sockaddr_un));
    socklen_t addrlen = sizeof(struct sockaddr_un);
    
    int new_port = accept(ctx.if_listen_port, (struct sockaddr *)&cliaddr, &addrlen);
    
    if(new_port > 0){
      if(ctx.if_session_port != 0){
        close(ctx.if_session_port);
      }
      
      printf("IF connected[%d]\n", new_port);
      ctx.if_session_port = new_port;
    }else{
      printf("accept error errno[%d]\n", errno);
    }
  }
}

static void proc_esp_spi()
{
  fd_set readFds;
  struct timeval tv;
  uint8_t  *data;
  uint32_t length;
  
  if(ctx.esp_st & ESP_DATA_MASK)
  {
    if(spi_recv_data(&data, &length)) {
      decoder_put(data, length);
    }
  }
}


static void proc_if()
{
  fd_set readFds;
  struct timeval tv;

  if(ctx.if_session_port == 0)
  {
    //tun socket disconnected
    return;
  }
  
  
  if((ctx.esp_st & ESP_BUFFER_FULL_MASK) != 0){
    //ESP RX buffer is full
    return;
  }
  
  
  FD_ZERO(&readFds);
  FD_SET(ctx.if_session_port, &readFds);
  
  tv.tv_sec  = 0;
  tv.tv_usec = READ_TIMEOUT_US;

  int len = select(ctx.if_session_port+1, &readFds, NULL, NULL, &tv);
  
  if(len == 0){
    //no data
    return;
  }
  
  if(len < 0){
    //tun socket is closed
    printf("IF closed\n");
    close(ctx.if_session_port);
    ctx.if_session_port = 0;
    return;
  }
  
  //Recv IPv4 packet header
  if(ctx.if_buffer_index < 4){
    ctx.if_buffer_index += read(ctx.if_session_port, ctx.if_buffer, 4-ctx.if_buffer_index);
    
    if(ctx.if_buffer_index < 4){
      return;
    }
    
    //0x45 is IPv4 packet type identifier
    if(ctx.if_buffer[0]==0x45){
      ctx.if_buffer_length = ntohs(*(uint16_t*)&ctx.if_buffer[2]);
    }else{
      printf("not ip packet¥n");
      //read remain data (invalid packet)
      read(ctx.if_session_port, ctx.if_buffer, sizeof(ctx.if_buffer));
      ctx.if_buffer_index = 0;
      return;
    }
  }
  
  //read remain data (valid IP packet)
  ctx.if_buffer_index += read(ctx.if_session_port, ctx.if_buffer+ctx.if_buffer_index,
    ctx.if_buffer_length - ctx.if_buffer_index);
  
  if(ctx.if_buffer_index >= ctx.if_buffer_length)
  {
    if(ctx.boot_seq == BOOT_END){
      send_esp(MSG_IP_PACKET, ctx.if_buffer, ctx.if_buffer_length);
    }
    ctx.if_buffer_index = 0;
  }
}

void sigpipe_handler(int unused)
{
  printf("SIGPIPE\n");
}

int main(int argc, uint8_t *argv[]){
  memset(&ctx, 0x00, sizeof(ctx));
  
  if(!init_esp_spi()){
    printf("init esp failed");
    return -1;
  }
  
  if(!init_if()){
    printf("init if failed");
    return -1;
  }
  
  //disable sigpipe default handler(default handler close app)
  signal(SIGPIPE, sigpipe_handler);
  
  while(1)
  {
    wait_if();
    
    if(!spi_read_st(&ctx.esp_st)){
      sleep(1);
      continue;
    }
   
    proc_esp_spi();
    
    proc_if();
  }
  
  spi_close();
  return 0;
}
