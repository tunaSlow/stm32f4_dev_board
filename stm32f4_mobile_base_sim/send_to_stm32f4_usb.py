import serial
import struct
import time

# Open serial port
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Wait for STM32 to initialize

# Dummy velocity values (in mm/s)
vx = 1234
vy = -567
vw = 890

# Pack data: mode=1, odom_cal=0, vx, vy, vw
packet = struct.pack('>BBhhh', 1, 0, vx, vy, vw)

# Pad to 64 bytes
packet += bytes(64 - len(packet))

# Send packet
ser.write(packet)

# Close serial port
ser.close()
