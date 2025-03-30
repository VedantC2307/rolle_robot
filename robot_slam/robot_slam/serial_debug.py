#!/usr/bin/env python3

import serial
import time

def main():
    # Configure the serial port
    port = '/dev/ttyS0'  # Default port
    baud = 115200        # Default baud rate
    
    print(f"Opening serial port {port} at {baud} baud...")
    
    try:
        # Open serial port
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=1.0
        )
        
        print(f"Serial port {port} opened successfully.")
        print("Reading data (press Ctrl+C to exit):")
        print("-" * 50)
        
        # Continuous reading loop
        while True:
            try:
                # Check if data is available
                if ser.in_waiting > 0:
                    # Read a line
                    line = ser.readline()
                    
                    # Try to decode as UTF-8 and print
                    try:
                        decoded = line.decode('utf-8').strip()
                        print(f"UTF-8: {decoded}")
                    except UnicodeDecodeError:
                        # If UTF-8 decoding fails, print raw bytes
                        print(f"Raw bytes: {line}")
                    
                    # Also print hex representation
                    print(f"HEX: {line.hex()}")
                    print("-" * 50)
                
                # Small delay to prevent CPU hogging
                time.sleep(0.01)
                
            except serial.SerialException as e:
                print(f"Serial error while reading: {e}")
                break
                
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Close the serial port if it's open
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
