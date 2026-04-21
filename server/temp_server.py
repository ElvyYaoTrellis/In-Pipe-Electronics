#!/usr/bin/env python3

import subprocess
import socket
import time

HOST = "0.0.0.0"  # Replace with local computer’s IP
PORT = 6000

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"Server listening on {socket.gethostbyname(socket.gethostname())}:{PORT} ...")

conn, addr = server_socket.accept()
print(f"Connected by {addr}")


while True:
    temp = subprocess.run(['cat', '/sys/class/thermal/thermal_zone1/temp'], capture_output=True).stdout.decode() # GPU TEMP, soc doesn't work for some reason
    # print(repr(temp))
    conn.sendall(temp.encode())

# if KeyboardInterrupt:
#     print("done")

# subprocess.run()

