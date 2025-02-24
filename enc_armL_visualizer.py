import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import serial
import time
from math import pi

class ArmL:
    def __init__(self):
        self.L1 = 30.4
        self.L2 = 28.05
        self.L3 = 4.4
        self.theta = np.array([-90, 90, 0, 0, 0, -90])
        self.d = np.array([0, 0, -self.L1, 0, -self.L2, 0])
        self.a = np.array([0, 0, 0, 0, 0, self.L3])
        self.alpha = np.array([-90, -90, 90, -90, 90, 0])
        self.joint_num = len(self.theta) + 1
        self.theta_c = []

    def DH(self, theta, d, a, alpha):
        theta = np.radians(theta)
        alpha = np.radians(alpha)
        dh_matrix = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        return dh_matrix

    def fk(self, theta_input):
        tip_0 = np.eye(4)
        self.theta_c = self.theta + theta_input
        for i in range(len(self.theta)):
            tip_0 = np.dot(tip_0, self.DH(self.theta[i] + theta_input[i], self.d[i], self.a[i], self.alpha[i]))
        return tip_0

    def get_joint_positions(self, theta_input):
        T = [np.eye(4)] * self.joint_num
        P = [np.zeros(3)] * self.joint_num
        R_matrices = [np.eye(3)] * self.joint_num

        T[0] = np.eye(4)
        P[0] = T[0][:3, 3]
        R_matrices[0] = T[0][:3, :3]

        self.theta_c = self.theta + theta_input
        for i in range(1, self.joint_num):
            T[i] = np.dot(T[i-1], self.DH(self.theta_c[i-1], self.d[i-1], self.a[i-1], self.alpha[i-1]))
            P[i] = T[i][:3, 3]
            R_matrices[i] = T[i][:3, :3]

        # Convert to numpy array and swap coordinates to match visualization
        P = np.array(P)
        P = np.column_stack((P[:, 2], P[:, 0], P[:, 1]))  # Swap coordinates
        
        return P, R_matrices

class ArmVisualizer:
    def __init__(self):
        # Initialize serial connection
        self.ser = serial.Serial('/dev/tty.usbmodem132938801', 115200)
        time.sleep(2)  # Wait for Arduino to initialize

        # Initialize ArmL
        self.arm = ArmL()
        
        # Create figure
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initialize plot elements
        self.line, = self.ax.plot([], [], [], 'b-', linewidth=2)
        self.joints, = self.ax.plot([], [], [], 'ro', markersize=8)
        
        # Set axis limits
        self.ax.set_xlim(-70, 70)
        self.ax.set_ylim(-70, 70)
        self.ax.set_zlim(-70, 70)
        
        # Labels
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('ArmL Real-time Visualization')

        # Initialize joint angles
        self.current_angles = np.zeros(6)

    def read_encoder_data(self):
        try:
            data = self.ser.readline().decode().strip()
            if data == "Invalid Angle":
                return None
            
            angle = float(data)  # Convert raw encoder value to degrees
            # Map the 16-bit encoder value (0-65535) to degrees (0-360)
            angle = (angle / 65535.0) * 360.0
            return angle
            
        except (ValueError, serial.SerialException) as e:
            print(f"Error reading encoder: {e}")
            return None

    def update(self, frame):
        # Read encoder data
        angle = self.read_encoder_data()
        if angle is not None:
            # Update the first joint angle (you can modify this to handle multiple encoders)
            self.current_angles[0] = angle

        # Calculate forward kinematics
        positions, _ = self.arm.get_joint_positions(self.current_angles)
        
        # Update visualization
        self.line.set_data(positions[:, 0], positions[:, 1])
        self.line.set_3d_properties(positions[:, 2])
        
        self.joints.set_data(positions[:, 0], positions[:, 1])
        self.joints.set_3d_properties(positions[:, 2])
        
        return self.line, self.joints

    def animate(self):
        ani = animation.FuncAnimation(
            self.fig, self.update,
            interval=50,  # 50ms refresh rate
            blit=True
        )
        plt.show()

    def cleanup(self):
        self.ser.close()
        print("Serial port closed")

def main():
    visualizer = ArmVisualizer()
    try:
        visualizer.animate()
    except KeyboardInterrupt:
        print("\nStopping visualization...")
    finally:
        visualizer.cleanup()

if __name__ == "__main__":
    main() 