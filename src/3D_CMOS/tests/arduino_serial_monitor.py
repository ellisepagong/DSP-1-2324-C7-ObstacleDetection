import serial
import time

# Adjust these as needed
COM_PORT = 'COM7'   # Your Arduino's COM port
BAUD_RATE = 9600    # Match this with your Arduino's Serial.begin()

def main():
    try:
        with serial.Serial(COM_PORT, BAUD_RATE, timeout=1) as ser:
            time.sleep(2)  # Wait for Arduino to reset (common on connect)
            print(f"Connected to {COM_PORT} at {BAUD_RATE} baud.")
            print("Reading serial data...\nPress Ctrl+C to stop.\n")
            
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    print(line)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nSerial monitor stopped by user.")

if __name__ == "__main__":
    main()
