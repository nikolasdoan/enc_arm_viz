import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import serial
import time
from math import pi

class ArmLR:
    def __init__(self, side='left'):
        self.side = side
        self.L1 = 4.4 
        self.L2 = 28.05
        self.L3 = 30.4
        
        """ This array defines the initial/home position angles (in degrees) for each joint 
        self.theta = np.array([-90, 90, 0, 0, 0, -90])
                                │   │  │  │  │   │
                                │   │  │  │  │   └─ Joint 6 (end-effector): -90° rotation
                                │   │  │  │  └─────  Joint 5: 0° rotation
                                │   │  │  └────────  Joint 4: 0° rotation
                                │   │  └───────────  Joint 3: 0° rotation
                                │   └──────────────  Joint 2: 90° rotation
                                └─────────────────── Joint 1 (base): -90° rotation
        """


        # Base theta values differ for left and right arms
        if side == 'left':
            self.theta = np.array([180, 90, 0, 0, 0, -90])
            self.origin = np.array([-20, 0, 0])  # -77.5 mm in Y direction
        else:  # right arm
            self.theta = np.array([180, -90, 0, 0, 0, -90])  # Mirrored angles
            self.origin = np.array([20, 0, 0])   # +77.5 mm in Y direction
            
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

    def get_joint_positions(self, theta_input):
        T = [np.eye(4)] * self.joint_num
        P = [np.zeros(3)] * self.joint_num
        R_matrices = [np.eye(3)] * self.joint_num

        # Initialize first transformation with origin offset
        T[0] = np.eye(4)
        T[0][:3, 3] = self.origin
        P[0] = T[0][:3, 3]
        R_matrices[0] = T[0][:3, :3]

        self.theta_c = self.theta + theta_input
        for i in range(1, self.joint_num):
            T[i] = np.dot(T[i-1], self.DH(self.theta_c[i-1], self.d[i-1], self.a[i-1], self.alpha[i-1]))
            P[i] = T[i][:3, 3]
            R_matrices[i] = T[i][:3, :3]

        P = np.array(P)
        P = np.column_stack((P[:, 2], P[:, 0], P[:, 1]))  # Swap coordinates
        
        return P, R_matrices

class DualArmVisualizer:
    def __init__(self, use_arduino=False):
        self.use_arduino = use_arduino
        
        # Initialize serial connection if using Arduino
        if use_arduino:
            try:
                self.ser = serial.Serial('/dev/tty.usbmodem132938801', 115200)
                time.sleep(2)
            except serial.SerialException:
                print("Arduino not connected. Switching to simulation mode.")
                self.use_arduino = False

        # Initialize both arms
        self.arm_left = ArmLR(side='left')
        self.arm_right = ArmLR(side='right')
        
        # Create figure
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initialize plot elements for both arms
        self.line_left, = self.ax.plot([], [], [], 'b-', linewidth=2, label='Left Arm')
        self.joints_left, = self.ax.plot([], [], [], 'bo', markersize=8)
        self.line_right, = self.ax.plot([], [], [], 'r-', linewidth=2, label='Right Arm')
        self.joints_right, = self.ax.plot([], [], [], 'ro', markersize=8)
        
        # Set axis limits
        self.ax.set_xlim(-70, 70)
        self.ax.set_ylim(-100, 100)  # Expanded Y limits to accommodate both arms
        self.ax.set_zlim(-70, 70)
        
        # Labels and legend
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('NIKO Dual Arm Real-time Visualization')
        self.ax.legend()

        # Initialize joint angles for both arms
        self.current_angles_left = np.zeros(6)
        self.current_angles_right = np.zeros(6)
        
        # Animation time for simulation mode
        self.t = 0

    def read_encoder_data(self):
        if not self.use_arduino:
            return None
            
        try:
            data = self.ser.readline().decode().strip()
            if data == "Invalid Angle":
                return None
            angle = float(data)
            angle = (angle / 65535.0) * 360.0
            return angle
        except (ValueError, serial.SerialException) as e:
            print(f"Error reading encoder: {e}")
            return None

    def simulate_movement(self):
        # Simulate simple sinusoidal movement for testing
        amplitude = 30
        frequency = 0.5
        angle = amplitude * np.sin(2 * np.pi * frequency * self.t)
        self.t += 0.05  # Time increment
        return angle

    def update(self, frame):
        if self.use_arduino:
            angle = self.read_encoder_data()
            if angle is not None:
                self.current_angles_left[0] = angle
                self.current_angles_right[0] = -angle  # Mirror the movement
        else:
            # Simulate movement when not using Arduino
            angle = self.simulate_movement()
            self.current_angles_left[0] = angle
            self.current_angles_right[0] = -angle

        # Calculate positions for both arms
        positions_left, _ = self.arm_left.get_joint_positions(self.current_angles_left)
        positions_right, _ = self.arm_right.get_joint_positions(self.current_angles_right)
        
        # Update visualization for left arm
        self.line_left.set_data(positions_left[:, 0], positions_left[:, 1])
        self.line_left.set_3d_properties(positions_left[:, 2])
        self.joints_left.set_data(positions_left[:, 0], positions_left[:, 1])
        self.joints_left.set_3d_properties(positions_left[:, 2])
        
        # Update visualization for right arm
        self.line_right.set_data(positions_right[:, 0], positions_right[:, 1])
        self.line_right.set_3d_properties(positions_right[:, 2])
        self.joints_right.set_data(positions_right[:, 0], positions_right[:, 1])
        self.joints_right.set_3d_properties(positions_right[:, 2])
        
        return self.line_left, self.joints_left, self.line_right, self.joints_right

    def animate(self):
        ani = animation.FuncAnimation(
            self.fig, self.update,
            interval=50,
            blit=True
        )
        plt.show()

    def cleanup(self):
        if self.use_arduino:
            self.ser.close()
            print("Serial port closed")

def main():
    # Set use_arduino=True to use real Arduino data, False for simulation
    visualizer = DualArmVisualizer(use_arduino=False)
    try:
        visualizer.animate()
    except KeyboardInterrupt:
        print("\nStopping visualization...")
    finally:
        visualizer.cleanup()

if __name__ == "__main__":
    main()
