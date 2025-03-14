import serial
import time

# Replace '/dev/ttyS0' with '/dev/ttyAMA0' if needed
SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

# Open the serial port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

print(f"Sending data to ESP32S3 on {SERIAL_PORT}...")

try:
    while True:
        message = "PWM:F:80\r\n"
        ser.write(message.encode())  # Send message
        print(f"Sent: {message.strip()}")
        time.sleep(2)  # Send message every 2 seconds

except KeyboardInterrupt:
    print("\nStopping...")
    ser.close()  # Close serial port when exiting
