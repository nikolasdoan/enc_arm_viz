import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import serial
import time
from math import pi

class ArmBase:
    def __init__(self, side='left'):
        self.L1 = 4.4 
        self.L2 = 28.05
        self.L3 = 30.4
        
        # Base theta values differ for left and right arms
        if side == 'left':
            self.theta = np.array([180, 90, 0, 0, 0, -90])
            self.origin = np.array([-20, 0, 0])  # Left arm origin
        else:  # right arm
            self.theta = np.array([180, -90, 0, 0, 0, -90])  # Mirrored angles
            self.origin = np.array([20, 0, 0])   # Right arm origin
            
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

        # Convert to numpy array and swap coordinates to match visualization
        P = np.array(P)
        P = np.column_stack((P[:, 2], P[:, 0], P[:, 1]))  # Swap coordinates
        
        return P, R_matrices

class DualArmVisualizer:
    def __init__(self):
        # Initialize both arms
        self.arm_left = ArmBase(side='left')
        self.arm_right = ArmBase(side='right')
        
        # Create figure
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initialize plot elements for both arms
        self.line_left, = self.ax.plot([], [], [], 'b-', linewidth=2, label='Left Arm')
        self.joints_left, = self.ax.plot([], [], [], 'bo', markersize=8)
        self.line_right, = self.ax.plot([], [], [], 'r-', linewidth=2, label='Right Arm')
        self.joints_right, = self.ax.plot([], [], [], 'ro', markersize=8)
        
        # Initialize path traces
        self.path_left, = self.ax.plot([], [], [], 'b--', alpha=0.5, label='Left Path')
        self.path_right, = self.ax.plot([], [], [], 'r--', alpha=0.5, label='Right Path')
        
        # Set axis limits
        self.ax.set_xlim(-70, 70)
        self.ax.set_ylim(-70, 70)
        self.ax.set_zlim(-70, 70)
        
        # Labels and legend
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Dual Arm Shape Drawing')
        self.ax.legend()

        # Initialize joint angles for both arms
        self.current_angles_left = np.zeros(6)
        self.current_angles_right = np.zeros(6)
        
        # Initialize shape paths
        self.left_path_points = self.generate_triangle_points()
        self.right_path_points = self.generate_square_points()
        
        # Animation time
        self.t = 0
        self.path_left_x = []
        self.path_left_y = []
        self.path_left_z = []
        self.path_right_x = []
        self.path_right_y = []
        self.path_right_z = []

    def generate_triangle_points(self):
        # Generate points for a triangle in front of the left arm
        center = np.array([-20, 40, 0])  # Center point for the triangle
        size = 15  # Size of the triangle
        points = np.array([
            center + [0, size, 0],  # Top point
            center + [-size, -size, 0],  # Bottom left
            center + [size, -size, 0],  # Bottom right
            center + [0, size, 0]  # Back to top to close the shape
        ])
        return points

    def generate_square_points(self):
        # Generate points for a square in front of the right arm
        center = np.array([20, 40, 0])  # Center point for the square
        size = 10  # Size of the square
        points = np.array([
            center + [-size, -size, 0],  # Bottom left
            center + [size, -size, 0],   # Bottom right
            center + [size, size, 0],    # Top right
            center + [-size, size, 0],   # Top left
            center + [-size, -size, 0]   # Back to start to close the shape
        ])
        return points

    def get_current_target_position(self, t, points):
        # Get the current target position along the path
        num_segments = len(points) - 1
        segment_duration = 1.0 / num_segments
        current_segment = int((t % 1.0) / segment_duration)
        segment_progress = (t % segment_duration) / segment_duration
        
        start = points[current_segment]
        end = points[(current_segment + 1) % len(points)]
        
        return start + (end - start) * segment_progress

    def update(self, frame):
        # Update time
        self.t += 0.01
        
        # Get current target positions for both arms
        left_target = self.get_current_target_position(self.t, self.left_path_points)
        right_target = self.get_current_target_position(self.t, self.right_path_points)
        
        # Simple motion - just for demonstration
        # In a real implementation, you would use inverse kinematics here
        self.current_angles_left = np.array([
            45 * np.sin(self.t), 
            45 * np.cos(self.t), 
            30 * np.sin(self.t * 2),
            20 * np.cos(self.t * 2),
            15 * np.sin(self.t * 3),
            10 * np.cos(self.t * 3)
        ])
        
        self.current_angles_right = np.array([
            -45 * np.sin(self.t),
            -45 * np.cos(self.t),
            -30 * np.sin(self.t * 2),
            -20 * np.cos(self.t * 2),
            -15 * np.sin(self.t * 3),
            -10 * np.cos(self.t * 3)
        ])

        # Calculate forward kinematics
        positions_left, _ = self.arm_left.get_joint_positions(self.current_angles_left)
        positions_right, _ = self.arm_right.get_joint_positions(self.current_angles_right)
        
        # Update arm visualizations
        self.line_left.set_data(positions_left[:, 0], positions_left[:, 1])
        self.line_left.set_3d_properties(positions_left[:, 2])
        self.joints_left.set_data(positions_left[:, 0], positions_left[:, 1])
        self.joints_left.set_3d_properties(positions_left[:, 2])
        
        self.line_right.set_data(positions_right[:, 0], positions_right[:, 1])
        self.line_right.set_3d_properties(positions_right[:, 2])
        self.joints_right.set_data(positions_right[:, 0], positions_right[:, 1])
        self.joints_right.set_3d_properties(positions_right[:, 2])
        
        # Update path traces
        end_effector_left = positions_left[-1]
        end_effector_right = positions_right[-1]
        
        self.path_left_x.append(end_effector_left[0])
        self.path_left_y.append(end_effector_left[1])
        self.path_left_z.append(end_effector_left[2])
        
        self.path_right_x.append(end_effector_right[0])
        self.path_right_y.append(end_effector_right[1])
        self.path_right_z.append(end_effector_right[2])
        
        # Update path visualizations
        self.path_left.set_data(self.path_left_x, self.path_left_y)
        self.path_left.set_3d_properties(self.path_left_z)
        self.path_right.set_data(self.path_right_x, self.path_right_y)
        self.path_right.set_3d_properties(self.path_right_z)
        
        return (self.line_left, self.joints_left, self.line_right, self.joints_right,
                self.path_left, self.path_right)

    def animate(self):
        ani = animation.FuncAnimation(
            self.fig, self.update,
            interval=50,  # 50ms refresh rate
            blit=True
        )
        plt.show()

def main():
    visualizer = DualArmVisualizer()
    try:
        visualizer.animate()
    except KeyboardInterrupt:
        print("\nStopping visualization...")

if __name__ == "__main__":
    main() 