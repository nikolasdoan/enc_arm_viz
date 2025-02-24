import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# Connect to Arduino
ser = serial.Serial('/dev/tty.usbmodem132938801', 115200)  # Match baudrate with Arduino
time.sleep(2)  # Wait for Arduino to initialize

# Create figure for plotting
fig, ax = plt.subplots()
ax.set_title('Encoder Angle Reading')
ax.set_xlabel('Time')
ax.set_ylabel('Angle (raw)')
ax.set_ylim(0, 65535)  # TLE5012 provides 16-bit angle readings

# Initialize data lists
max_points = 100
xs = list(range(max_points))
ys = [0] * max_points

# Create line object with initial data
line, = ax.plot(xs, ys)

# Animation update function
def update(frame):
    try:
        # Read data from Arduino
        data = ser.readline().decode().strip()
        
        if data == "Invalid Angle":
            print("Invalid angle reading")
            return line,
        
        try:
            angle = int(data)
            
            # Add new data to lists
            ys.pop(0)
            ys.append(angle)
            
            # Update line data
            line.set_ydata(ys)
            
            # Adjust plot limits if needed
            if min(ys) < ax.get_ylim()[0] or max(ys) > ax.get_ylim()[1]:
                ax.relim()
                ax.autoscale_view()
                
        except ValueError:
            print(f"Invalid data received: {data}")
            
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        plt.close()
        return line,
        
    return line,

# Create animation
ani = animation.FuncAnimation(
    fig, update, 
    interval=50,  # Update every 50ms
    blit=True
)

# Enable grid
plt.grid(True)

try:
    plt.show()
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    ser.close()
    print("Serial port closed")
