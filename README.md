INSTALLATION INSTRUCTIONS:

Here are the required Python packages you'll need to install:

1. `pyserial` - for serial communication with Arduino
2. `numpy` - for numerical computations
3. `matplotlib` - for visualization and plotting

The main package you're missing is likely `pyserial` (the others usually come with Python data science distributions like Anaconda). You can install it using pip:

```bash
pip install pyserial
```

If you're starting from scratch, you can install all required packages with:

```bash
pip install pyserial numpy matplotlib
```

Note: 
- If you're using Anaconda, you might want to use `conda` instead:
  ```
  conda install pyserial numpy matplotlib
  ```
- Make sure to check that your Arduino is connected at the correct port. In both files, it's set to `/dev/tty.usbmodem132938801`, which is a macOS port. If you're on:
  - Windows: It would be something like `COM3` or `COM4`
  - Linux: It would be something like `/dev/ttyUSB0` or `/dev/ttyACM0`

You'll need to modify the serial port in both files to match your system's port name.


ROBOT ARM VISUALIZATION PARAMETERS:


This array defines the initial/home position angles (in degrees) for each joint of the robotic arm, where each value corresponds to a specific joint from base to end-effector:

```python NIKO_armLR_visualizer.py
self.theta = np.array([-90, 90, 0, 0, 0, -90])
                        │   │  │  │  │   │
                        │   │  │  │  │   └─ Joint 6 (end-effector): -90° rotation
                        │   │  │  │  └─────  Joint 5: 0° rotation
                        │   │  │  └────────  Joint 4: 0° rotation
                        │   │  └───────────  Joint 3: 0° rotation
                        │   └──────────────  Joint 2: 90° rotation
                        └─────────────────── Joint 1 (base): -90° rotation
```

These angles, combined with the DH parameters (d, a, alpha), define the arm's home configuration:
- The first -90° rotation at the base orients the arm
- The second 90° rotation helps position the arm in its starting pose
- The middle joints (3,4,5) start at 0°
- The final -90° sets the end-effector orientation

When the arm moves, these base angles are added to the input angles (`theta_input`) to get the final joint angles (`self.theta_c = self.theta + theta_input`).




The DH (Denavit-Hartenberg) parameters are a systematic method to describe the kinematics of a robot arm. Let me break down each parameter array:

```python:NIKO_armLR_visualizer.py
# d: Joint offset (distance along previous z to common normal)
self.d = np.array([0, 0, -self.L1, 0, -self.L2, 0])
#                  │  │    │      │    │      │
#                  │  │    │      │    │      └─ Joint 6: No offset
#                  │  │    │      │    └────────  Joint 5: -L2 offset (28.05mm)
#                  │  │    │      └─────────────  Joint 4: No offset
#                  │  │    └────────────────────  Joint 3: -L1 offset (30.4mm)
#                  │  └───────────────────────── Joint 2: No offset
#                  └──────────────────────────── Joint 1: No offset

# a: Link length (distance along common normal)
self.a = np.array([0, 0, 0, 0, 0, self.L3])
#                  │  │  │  │  │    │
#                  │  │  │  │  │    └─ Joint 6: L3 length (4.4mm)
#                  │  │  │  │  └──────  Joint 5: No length
#                  │  │  │  └─────────  Joint 4: No length
#                  │  │  └────────────  Joint 3: No length
#                  │  └───────────────  Joint 2: No length
#                  └──────────────────  Joint 1: No length

# alpha: Link twist (rotation about common normal)
self.alpha = np.array([-90, -90, 90, -90, 90, 0])
#                      │    │    │    │    │   │
#                      │    │    │    │    │   └─ Joint 6: No twist
#                      │    │    │    │    └───── Joint 5: 90° twist
#                      │    │    │    └────────── Joint 4: -90° twist
#                      │    │    └───────────────  Joint 3: 90° twist
#                      │    └──────────────────── Joint 2: -90° twist
#                      └─────────────────────────  Joint 1: -90° twist
```

These parameters define how each joint connects to the next:
- `d`: The offset along the z-axis between links
- `a`: The length of the link along the x-axis
- `alpha`: The twist angle between consecutive z-axes
- (theta): The joint angle (defined separately in `self.theta`)

Together with the joint angles (theta), these parameters are used in the DH transformation matrix to calculate the position and orientation of each joint in the robot arm chain.

The DH matrix combines these parameters as shown in the `DH()` method:
```python
def DH(self, theta, d, a, alpha):
    theta = np.radians(theta)
    alpha = np.radians(alpha)
    dh_matrix = np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,             np.sin(alpha),                np.cos(alpha),                d              ],
        [0,             0,                            0,                            1              ]
    ])
    return dh_matrix
```

This matrix represents the complete transformation from one joint to the next, incorporating both rotation and translation.



In `NIKO_armLR_visualizer.py`, the revolute joints are represented through the DH parameters and joint angles. Let me explain how:

1. The joint angles (revolute joints) are primarily represented in:

````python:NIKO_armLR_visualizer.py
# Initial joint angles (home position)
if side == 'left':
    self.theta = np.array([-90, 90, 0, 0, 0, -90])  # 6 revolute joints
    self.origin = np.array([0, -77.5, 0])
else:  # right arm
    self.theta = np.array([90, -90, 0, 0, 0, 90])   # 6 revolute joints
    self.origin = np.array([0, 77.5, 0])
````

2. The movement of these revolute joints is handled in:

````python:NIKO_armLR_visualizer.py
def get_joint_positions(self, theta_input):
    # ... existing code ...
    
    # This line updates the current joint angles by adding input angles
    self.theta_c = self.theta + theta_input
    
    # Calculate each joint's transformation
    for i in range(1, self.joint_num):
        T[i] = np.dot(T[i-1], self.DH(self.theta_c[i-1], self.d[i-1], self.a[i-1], self.alpha[i-1]))
        P[i] = T[i][:3, 3]  # Joint position
        R_matrices[i] = T[i][:3, :3]  # Joint orientation
````

3. The actual joint movement simulation/reading happens in:

````python:NIKO_armLR_visualizer.py
def update(self, frame):
    if self.use_arduino:
        angle = self.read_encoder_data()
        if angle is not None:
            self.current_angles_left[0] = angle  # Update first joint
            self.current_angles_right[0] = -angle  # Mirror for right arm
    else:
        # Simulate movement
        angle = self.simulate_movement()
        self.current_angles_left[0] = angle
        self.current_angles_right[0] = -angle
````

For comparison with the URDF file, these joints correspond to:
```xml:humanoid.urdf
<joint name="1" type="revolute">  <!-- Base joint -->
<joint name="2" type="revolute">  <!-- Shoulder joint -->
<joint name="3" type="revolute">  <!-- Upper arm joint -->
<joint name="4" type="revolute">  <!-- Elbow joint -->
<joint name="5" type="revolute">  <!-- Forearm joint -->
<joint name="6" type="revolute">  <!-- Wrist joint -->
```

Would you like me to add explicit joint limit parameters or additional joint properties to the visualization code to match the URDF specifications?
