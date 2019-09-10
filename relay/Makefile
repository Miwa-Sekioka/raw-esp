#Makefile
# ESP8266 raw-esp host program

server: wroom_relay.c esp_spi.c utility.c
	gcc -o wroom_relay  wroom_relay.c crc16.c esp_spi.c utility.c

