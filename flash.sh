esptool.py --port /dev/ttyUSB0 --baud 460800 write_flash --flash_size=detect 0 ../bin/boot_v1.2.bin 0x1000 ../bin/upgrade/user1.2048.new.3.bin 0x1fc000 ../bin/esp_init_data_default_v08.bin 0x1FB000 ../bin/blank.bin 0x1fE000 ../bin/blank.bin

