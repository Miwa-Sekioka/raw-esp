#!/bin/bash

while :
do
  socat -d -d /tmp/server.sock TUN:192.168.4.1/24,iff-no-pi,iff-up,tun-name=wroom_tun &
  ifconfig wroom_tun mtu 1200
  wait %1
  sleep 5
done
 
