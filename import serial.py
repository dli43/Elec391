import serial
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# === Serial Config ===
ser = serial.Serial('COM3', baudrate=9600, timeout=1)
time.sleep(2)  # Allow Arduino time to reset

x_vals, y_vals, z_vals = [], [], []
max_points = 50

def get_accel_data():
    line = ser.readline().decode().strip()
    if line and ',' in line:
        try:
            x_str, y_str, z_str = line.split(',')
            return float(x_str), float(y_str), float(z_str)
        except ValueError:
            return None
    return None

# === Plot Update Function ===
def update(frame):
    data = get_accel_data()
    if data:
        x, y, z = data
        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)

        if len(x_vals) > max_points:
            x_vals.pop(0)
            y_vals.pop(0)
            z_vals.pop(0)

        plt.cla()
        plt.plot(x_vals, label='X')
        plt.plot(y_vals, label='Y')
        plt.plot(z_vals, label='Z')
        plt.ylim(-10, 10)  # Adjust depending on motion
        plt.title("Real-Time Accelerometer Data")
        plt.ylabel("Acceleration (m/s²)")
        plt.xlabel("Samples")
        plt.legend(loc='upper right')
        plt.tight_layout()

ani = FuncAnimation(plt.gcf(), update, interval=100)  # Update every 100ms
plt.show()
