import serial
import csv
import time

# --- Configuration ---
PORT = 'COM5'          # Change to match your Arduino port (e.g., '/dev/ttyUSB0' for Linux/Mac)
BAUDRATE = 115200
FILENAME = 'arduino_log.csv'
TIME_LIMIT = 60        # Set to None to log indefinitely

def log_serial_to_csv(port, baudrate, filename, time_limit=None):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser, open(filename, 'w', newline='') as csvfile:
            print(f"Connected to {port} at {baudrate} baud.")
            print(f"Logging to {filename}...")

            writer = csv.writer(csvfile)
            start_time = time.time()

            # Give Arduino time to reset
            time.sleep(2)

            while True:
              if ser.in_waiting > 0:
                  line = ser.readline().decode('utf-8', errors='replace').strip()
                  print(line)  # Print to terminal
                  writer.writerow([line])  # Log entire line as one CSV cell

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Logging stopped by user.")

if __name__ == "__main__":
    log_serial_to_csv(PORT, BAUDRATE, FILENAME, TIME_LIMIT)