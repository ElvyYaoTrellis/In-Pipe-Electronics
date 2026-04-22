#!/bin/bash

# Kill anything using relevant ports
sudo fuser -k 8005/tcp 2>/dev/null
sudo fuser -k 8080/tcp 2>/dev/null
sudo fuser -k 8081/tcp 2>/dev/null
sudo fuser -k 8554/tcp 2>/dev/null

sleep 1

# Start rkaiq ISP daemon (required before any camera pipeline)
sudo rkaiq_3A_server > /dev/null 2>&1 &
sleep 2

sudo python3 led_server.py &
sudo python3 rtsp_dual.py > /tmp/rtsp.log 2>&1 &
sudo python3 motor_velocity_current.py --server --host 0.0.0.0 --port 8005 --ina-addr 0x40 --ina-shunt-ohms 0.05 --ina-max-current-a 5 --log-csv /tmp/telemetry.csv
