import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.widgets import Slider
import serial
import time
from math import pi

class ArmLR:
    def __init__(self, side='left'):
        self.side = side
        self.L1 = 4.4 
        self.L2 = 28.05
        self.L3 = 30.4        
        # Base theta values differ for left and right arms
        if side == 'left':
            self.theta = np.array([180, 90, 0, 0, 0, -90])
            self.origin = np.array([-20, 0, 0])  # -20 mm in Y direction
        else:  # right arm
            self.theta = np.array([180, -90, 0, 0, 0, -90])  # Mirrored angles
            self.origin = np.array([20, 0, 0])   # +20 mm in Y direction
            
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
        
        # Create figure with space for sliders
        self.fig = plt.figure(figsize=(15, 8))
        
        # Create grid spec to organize the layout
        gs = self.fig.add_gridspec(1, 3, width_ratios=[1, 3, 1])
        
        # Create main 3D plot in the center
        self.ax = self.fig.add_subplot(gs[0, 1], projection='3d')
        
        # Create slider axes
        self.slider_ax_left = []
        self.slider_ax_right = []
        self.sliders_left = []
        self.sliders_right = []
        
        # Create sliders for left arm on the left side
        slider_panel_left = self.fig.add_subplot(gs[0, 0])
        slider_panel_left.set_title('Left Arm Controls')
        slider_panel_left.axis('off')
        for i in range(6):
            ax = self.fig.add_axes([0.05, 0.8 - i*0.1, 0.2, 0.02])
            slider = Slider(ax, f'J{i+1}L', -180, 180, valinit=0)
            self.slider_ax_left.append(ax)
            self.sliders_left.append(slider)
            slider.on_changed(self.update_sliders)
            
        # Create sliders for right arm on the right side
        slider_panel_right = self.fig.add_subplot(gs[0, 2])
        slider_panel_right.set_title('Right Arm Controls')
        slider_panel_right.axis('off')
        for i in range(6):
            ax = self.fig.add_axes([0.75, 0.8 - i*0.1, 0.2, 0.02])
            slider = Slider(ax, f'J{i+1}R', -180, 180, valinit=0)
            self.slider_ax_right.append(ax)
            self.sliders_right.append(slider)
            slider.on_changed(self.update_sliders)
        
        # Initialize plot elements for both arms
        self.line_left, = self.ax.plot([], [], [], 'b-', linewidth=2, label='Left Arm')
        self.joints_left, = self.ax.plot([], [], [], 'bo', markersize=8)
        self.line_right, = self.ax.plot([], [], [], 'r-', linewidth=2, label='Right Arm')
        self.joints_right, = self.ax.plot([], [], [], 'ro', markersize=8)
        
        # Set axis limits
        self.ax.set_xlim(-70, 70)
        self.ax.set_ylim(-100, 100)
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
        
        # Initial update
        self.update_robot()

    def update_sliders(self, val):
        # Update current angles from sliders
        for i in range(6):
            self.current_angles_left[i] = self.sliders_left[i].val
            self.current_angles_right[i] = self.sliders_right[i].val
        self.update_robot()

    def update_robot(self):
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
        
        self.fig.canvas.draw_idle()

    def animate(self):
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
