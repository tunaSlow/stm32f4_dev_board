import serial
import time

# Configuration
PORT = '/dev/ttyUSB0'
BAUD = 115200

def main():
    try:
        # Open the serial port
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"Connected to {PORT} at {BAUD} baud.")
        
        # Give the CH340 chip a moment to stabilize after opening the port
        time.sleep(1)

        counter = 0.0
        
        while True:
            # Create a formatted string matching your STM32 sscanf logic
            # Example: <1.50,2.50,3.50>\n
            payload = f"<{counter:.2f},{counter+1.0:.2f},{counter+2.0:.2f}>\n"
            
            # Send the encoded bytes
            ser.write(payload.encode('utf-8'))
            print(f"Sent: {payload.strip()}")
            
            counter += 0.5
            time.sleep(2)  # Wait 2 seconds before sending the next payload

    except serial.SerialException as e:
        print(f"Error opening or writing to serial port: {e}")
    except KeyboardInterrupt:
        print("\nExiting test script.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()