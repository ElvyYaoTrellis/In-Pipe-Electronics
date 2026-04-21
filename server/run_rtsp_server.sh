# # ./rtsp_server.py --device /dev/video0 --width 1920 --height 1080 --fps 60 --bitrate 20000000



# #!/bin/bash
# # Start all services: RTSP, servo server, LED server

# RADXA_DEV="/dev/video0"
# WIDTH=1920
# HEIGHT=1080
# FPS=60
# BITRATE=20000000

# # Start RTSP server
# echo "[RTSP] Starting..."
# ./rtsp_server.py \
#   --device "$RADXA_DEV" \
#   --width $WIDTH \
#   --height $HEIGHT \
#   --fps $FPS \
#   --bitrate $BITRATE \
#   > rtsp.log 2>&1 &

# # Start Servo server (port 80) – needs sudo if not using setcap
# echo "[Servo] Starting..."
# sudo -E python3 servo_server.py > servo.log 2>&1 &

# # Start LED server (port 8080)
# echo "[LED] Starting..."
# sudo -E python3 led_server.py > led.log 2>&1 &

# echo "All servers launched. Logs: rtsp.log, servo.log, led.log"

# sudo python3 rtsp_server.py --device /dev/video0 --width 1920 --height 1080 --fps 60 --bitrate 20000000
# sudo python3 led_server.py
# sudo python3 servo_server.py

#!/bin/bash

# Kill anything using ports 80, 8080, 8554
sudo fuser -k 8005/tcp   2>/dev/null
sudo fuser -k 8080/tcp 2>/dev/null
sudo fuser -k 5000/tcp 2>/dev/null
sudo fuser -k 8181/tcp 2>/dev/null


# Small pause to ensure processes are released
sleep 1

# sudo python3 rtsp_server.py & #--device /dev/video0 --width 1280 --height 720 --fps 20 --bitrate 20000000 &
# sudo python3 temp_server.py &
#sudo python3 IMU_server.py &
# sudo python3 cam2_server.py &
sudo python3 led_server.py &
# sudo python3 servo_server.py &
sudo python3 udp_dual.py --host 192.168.8.223 --port 5000 &

# sudo python3 2Dac2Motor_network.py --server --host 0.0.0.0 --port 8000 
# sudo python3 motor_velocity.py --server --host 0.0.0.0 --port 8005
sudo python3 motor_velocity_current.py --server --host 0.0.0.0 --port 8005 --ina-addr 0x40 --ina-shunt-ohms 0.05 --ina-max-current-a 5 --log-csv /tmp/telemetry.csv



# sudo python3 IMU_server.py

