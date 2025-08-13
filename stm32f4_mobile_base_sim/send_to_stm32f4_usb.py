import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

message = "Hello STM32"  # Add newline to trigger receive
ser.write(message.encode())

ser.close()
