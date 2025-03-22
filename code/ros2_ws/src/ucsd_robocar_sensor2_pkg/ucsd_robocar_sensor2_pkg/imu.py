import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Define the port - change as needed
SERIAL_PORT = 'ttyACM0'  # or ttyUSB0, etc.
BAUD_RATE = 115200

# Threshold values for drift detection
YAW_RATE_THRESHOLD = 0.5  # rad/s
LATERAL_ACCEL_THRESHOLD = 0.3  # G

# Data storage
time_values = []
gyro_data = {"x": [], "y": [], "z": []}
accel_data = {"x": [], "y": [], "z": []}

# Initialize serial connection
def read_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        start_time = time.time()
        
        while True:
            if ser.in_waiting:
                data = ser.readline().decode('utf-8', errors='replace').strip()
                elapsed_time = time.time() - start_time
                print(f"{elapsed_time:.3f}, {data}")
                
                try:
                    values = data.split(',')
                    if len(values) >= 6:  # Adjust based on actual format
                        gyro_x = float(values[0])
                        gyro_y = float(values[1])
                        gyro_z = float(values[2])
                        accel_x = float(values[3])
                        accel_y = float(values[4])
                        accel_z = float(values[5])

                        time_values.append(elapsed_time)
                        gyro_data["x"].append(gyro_x)
                        gyro_data["y"].append(gyro_y)
                        gyro_data["z"].append(gyro_z)
                        accel_data["x"].append(accel_x)
                        accel_data["y"].append(accel_y)
                        accel_data["z"].append(accel_z)

                        if abs(gyro_z) > YAW_RATE_THRESHOLD and abs(accel_y) > LATERAL_ACCEL_THRESHOLD:
                            print(f"{elapsed_time:.3f}, DRIFT DETECTED!")
                except:
                    pass
            
            time.sleep(0.01)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")

# Plot data
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

# Update function for animation
def update_plot(frame):
    ax1.clear()
    ax2.clear()
    
    if time_values:
        ax1.plot(time_values, gyro_data["x"], label="Gyro X", color="r")
        ax1.plot(time_values, gyro_data["y"], label="Gyro Y", color="g")
        ax1.plot(time_values, gyro_data["z"], label="Gyro Z", color="b")
        ax1.set_title("Gyroscope Data")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Rotation Rate (rad/s)")
        ax1.legend()

        ax2.plot(time_values, accel_data["x"], label="Accel X", color="r")
        ax2.plot(time_values, accel_data["y"], label="Accel Y", color="g")
        ax2.plot(time_values, accel_data["z"], label="Accel Z", color="b")
        ax2.set_title("Acceleration Data")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Acceleration (G)")
        ax2.legend()
    
ani = animation.FuncAnimation(fig, update_plot, interval=500)

if __name__ == "__main__":
    import threading
    thread = threading.Thread(target=read_serial, daemon=True)
    thread.start()
    plt.show()
