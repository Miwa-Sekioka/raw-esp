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
#include "message.h"
#include "crc16.h"
#include <stdbool.h>
#include <errno.h>
#include <signal.h>
#include "esp_spi.h"
#include "utility.h"

#define UNIXDOMAIN_PATH "/tmp/server.sock"
#define SPI_DEVICE  "/dev/spidev0.0"
#define ESP_IP_ADDR "192.168.4.1"
#define ESP_IP_MASK "255.255.255.0"
#define ESP_IP_DHCP_L "192.168.4.10"
#define ESP_IP_DHCP_H "192.168.4.20"
#define ESP_SSID "FACE-LOCK"
#define ESP_PASS "miwa3069"
#define READ_BUFFER_SIZE 2048
#define READ_TIMEOUT_US 1000
#define MAX_DATA_SIZE 1500
#define WAIT_BOOT_INTERVAL 500000



//Prototype
typedef void (*comm_callback_t)(uint8_t type, uint8_t *data, uint32_t len);
static void packet_from_host(uint8_t type, uint8_t *data, uint32_t len);
static bool send_esp(uint8_t type, uint8_t *data, uint32_t len);
static bool init_esp();
static bool init_if();
static void on_msg_boot(uint8_t *data, uint32_t len);
static void on_msg_status(uint8_t *data, uint32_t len);
static void on_msg_log(uint8_t *data, uint32_t len);
static void on_msg_ip(uint8_t *data, uint32_t len);

//Data
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
  
  //IF側バッファ
  uint8_t if_buffer[READ_BUFFER_SIZE];
  uint32_t if_buffer_index;
  uint32_t if_buffer_length;
  
  //最後に取得したSPIステータス
  uint8_t esp_st;
};
struct app_context ctx;

static bool send_esp(uint8_t type, uint8_t *data, uint32_t len)
{
  uint8_t buf[len+3];
  uint16_t crc;
  
  //送信フレームを作成
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
  
  printf(">>:%02x:%d\n", type, len+3);//type+crc=3bytes
  
  return spi_send_data(buf, len+3);
}

//Function
static void packet_from_host(uint8_t type, uint8_t *data, uint32_t len)
{
  printf("<<:%02x:%d\n", type, len);
  
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
  uint8_t cmd = 40;//info
  printf("log level=%d\n", cmd);
  ctx.boot_seq = BOOT_LOG_LEVEL_SET;  
  send_esp(MSG_LOG_LEVEL_SET, &cmd, 1);
}

static void on_msg_log(uint8_t *data, uint32_t len)
{
  int i;
  printf("LOG:");
  for(i=0; i<len; i++){
    //表示できない文字は無視
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
      struct msg_softap_conf cmd;
      
      printf("ssid=%s,pass=%s\n", ESP_SSID, ESP_PASS);
      memset(&cmd, 0x00, sizeof(cmd));
      cmd.ssid_len = strlen(ESP_SSID);
      memcpy(cmd.ssid, ESP_SSID, strlen(ESP_SSID));
      cmd.password_len = strlen(ESP_PASS);
      memcpy(cmd.password, ESP_PASS, strlen(ESP_PASS));
      cmd.channel = 1;
      cmd.auth_mode = WIFI_AUTH_WPA2_PSK;
      cmd.beacon_interval = 500;// 500 * 1/1024ms
      
      ctx.boot_seq = BOOT_SOFTAP_CONF_SET;
      send_esp(MSG_SOFTAP_CONF_SET, (uint8_t*)&cmd, sizeof(cmd));
    }break;
    case BOOT_SOFTAP_CONF_SET:{
      printf("APIP=%s DHCP=enable\n", ESP_IP_ADDR);
      struct msg_softap_net_conf cmd;
      memset(&cmd, 0x00, sizeof(cmd));
      cmd.address = htonl(ipstr_to_uint32(ESP_IP_ADDR));
      cmd.netmask = htonl(ipstr_to_uint32(ESP_IP_MASK));
      cmd.gateway = 0;
      cmd.enable_dhcpd = 1;
      cmd.dhcpd_offer_gateway = 1;
      cmd.dhcpd_first_ip = htonl(ipstr_to_uint32(ESP_IP_DHCP_L));
      cmd.dhcpd_last_ip  = htonl(ipstr_to_uint32(ESP_IP_DHCP_H));
      
      ctx.boot_seq = BOOT_SOFTAP_NET_CONF_SET;
      send_esp(MSG_SOFTAP_NET_CONF_SET, (uint8_t*)&cmd, sizeof(cmd));
    }break;
    case BOOT_SOFTAP_NET_CONF_SET:{
      ctx.boot_seq = BOOT_END;
      printf("BOOT Complete\n");
    }break;
  }
}

static inline void on_frame_received(uint8_t *data, uint32_t len)
{
  uint16_t crc_msg;
  uint16_t crc_calc;

  if (len < 3) {
    printf("len err\n");
    return;
  }

  crc_calc = crc16_block(data, len - 2);
  memcpy(&crc_msg, data + len - 2, 2);
  if (crc_calc != crc_msg) {
    printf("crc err %04x %04x\n", crc_msg, crc_calc);
    return;
  }

  packet_from_host(data[0], data + 1, len - 3);
}

static bool init_esp_spi(uint8_t *device)
{
  if(!spi_init(device)){
    return false;
  }
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

  //delete file
  unlink(UNIXDOMAIN_PATH);
  
  //bind unix socket
  memset(&srvaddr, 0, sizeof(struct sockaddr_un));
  srvaddr.sun_family = AF_UNIX;
  strcpy(srvaddr.sun_path, UNIXDOMAIN_PATH);
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
    if (spi_recv_data(&data, &length)) {
      on_frame_received(data, length);
    }
  }
}


static void proc_if()
{
  fd_set readFds;
  struct timeval tv;

  
  if((ctx.esp_st & ESP_BUFFER_FULL_MASK) != 0){
    //ESPのバッファが満状態、送信可能になるまで待つ
    return;
  }
  
  //起動が未完了
  if(ctx.boot_seq <= BOOT_LOG_LEVEL_SET)
  {
    //初期化シーケンス開始
    on_msg_boot(NULL, 0);
    //複数回送らないように待機
    usleep(WAIT_BOOT_INTERVAL_US);
    return;
  }
  
  if(ctx.if_session_port == 0)
  {
    //IF未接続
    return;
  }
  
  //シリアルポート待機時間
  tv.tv_sec  = 0;
  tv.tv_usec = READ_TIMEOUT_US;
  
  FD_ZERO(&readFds);
  FD_SET(ctx.if_session_port, &readFds);

  int len = select(ctx.if_session_port+1, &readFds, NULL, NULL, &tv);
  
  if(len == 0){
    //データが無い
    return;
  }
  
  if(len < 0){
    //UNIXソケットが閉じられている
    printf("IF closed\n");
    close(ctx.if_session_port);
    ctx.if_session_port = 0;
    return;
  }
  
  //IPv4ヘッダの読み込み（lengthまで）
  if(ctx.if_buffer_index < 4){
    ctx.if_buffer_index += read(ctx.if_session_port, ctx.if_buffer, 4-ctx.if_buffer_index);
    
    //不足分が受信されるまで、待つ
    if(ctx.if_buffer_index < 4){
      return;
    }
    
    //0x45はIPv4パケットの識別ヘッダ
    if(ctx.if_buffer[0]==0x45){
      ctx.if_buffer_length = ntohs(*(uint16_t*)&ctx.if_buffer[2]);
    }else{
      printf("not ip packet¥n");
      //残り読み込み
      read(ctx.if_session_port, ctx.if_buffer, sizeof(ctx.if_buffer));
      ctx.if_buffer_index = 0;
      return;
    }
  }
  
  //残り読み込み
  ctx.if_buffer_index += read(ctx.if_session_port, ctx.if_buffer+ctx.if_buffer_index,
    ctx.if_buffer_length - ctx.if_buffer_index);
  
  if(ctx.if_buffer_index >= ctx.if_buffer_length)
  {
    //設定完了していなければ、送信しない
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
  
  uint8_t *device = SPI_DEVICE;
  
  if(argc > 1){
    device = argv[1];
  }
  
  if(!init_esp_spi(device)){
    printf("init esp failed");
    return -1;
  }
  
  if(!init_if()){
    printf("init if failed");
    return -1;
  }
  //IF側が切断されても、終了しないようにする
  signal(SIGPIPE, sigpipe_handler);
  usleep(1000);
  
  while(1)
  {
    wait_if();
    
    if(!spi_read_st(&ctx.esp_st)){
      sleep(1);
      continue;
    }
   
    //ESP処理
    proc_esp_spi();
    
    //IP処理
    proc_if();
  }
  
  spi_close();
  return 0;
}


