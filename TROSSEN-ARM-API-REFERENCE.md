## 📘 **Comprehensive API Reference - Trossen Arm Python Bindings**

> **Note:**  
> This was generated using Claude Sonnet 4.5 on the trossen-arm.pyi

## **1. Core Control Modes** (`Mode` Enum)

The robot operates in different control modes that determine how each joint behaves:

| Mode | Description | Use Case |
|------|-------------|----------|
| **`idle`** | All joints braked, motors holding position | Safety/standby state |
| **`position`** | Position control with trajectory tracking | Pick-and-place, waypoint following |
| **`velocity`** | Velocity control | Continuous motion tasks |
| **`external_effort`** | Force/torque control | Gravity compensation, compliant motion |
| **`effort`** | Direct torque/force commands | Low-level motor control |

---

## **2. Robot Models** (`Model` Enum)

Three robot variants are supported:

- **`wxai_v0`** - WXAI V0 model (6-DOF arm + gripper)
- **`vxai_v0_right`** - VXAI V0 Right-handed variant
- **`vxai_v0_left`** - VXAI V0 Left-handed variant

---

## **3. End Effector Configurations** (`StandardEndEffector`)

Pre-configured end-effector properties:

- **`no_gripper`** - Arm without gripper
- **`wxai_v0_base`** - WXAI base gripper
- **`wxai_v0_leader`** - Leader arm gripper (for teleoperation)
- **`wxai_v0_follower`** - Follower arm gripper (for teleoperation)
- **`vxai_v0_base`** - VXAI base gripper

### **`EndEffector` Class Properties**
```python
end_effector.t_flange_tool         # 4x4 transformation matrix (flange → tool)
end_effector.mass                  # End-effector mass (kg)
end_effector.palm                  # Gripper palm width (m)
end_effector.finger_left           # Left finger length (m)
end_effector.finger_right          # Right finger length (m)
end_effector.pitch_circle_radius   # Pitch circle radius (m)
end_effector.offset_finger_left    # Left finger offset (m)
end_effector.offset_finger_right   # Right finger offset (m)
```

---

## **4. Interpolation Spaces** (`InterpolationSpace`)

Defines how trajectories are interpolated:

- **`joint`** - Joint space interpolation (straight line in joint angles)
- **`cartesian`** - Cartesian space interpolation (straight line in 3D space)

---

## **5. TrossenArmDriver Main Class**

### **A. Initialization & Configuration**

```python
# Create driver instance
driver = trossen_arm.TrossenArmDriver()

# Configure the robot
driver.configure(
    model=trossen_arm.Model.wxai_v0,
    end_effector=trossen_arm.StandardEndEffector.wxai_v0_leader,
    serv_ip="192.168.1.2",
    clear_error=False,
    timeout=20.0  # seconds
)

# Check configuration status
is_configured = driver.get_is_configured()  # Returns bool

# Cleanup (reboot optional)
driver.cleanup(reboot_controller=False)
```

### **B. Position Control**

**Joint Space:**
```python
# Set all joints
from math import pi
driver.set_all_positions(
    goal_positions=[0.0, pi/2, pi/2, 0.0, 0.0, 0.0, 0.0], # rad or m
    goal_time=2.0,                                        # seconds
    blocking=True,                                        # wait for completion
    goal_feedforward_velocities=None,                     # optional
    goal_feedforward_accelerations=None                   # optional
)

# Set arm joints only (excludes gripper)
driver.set_arm_positions(goal_positions, goal_time, blocking)

# Set single joint
driver.set_joint_position(joint_index=0, goal_position=0.5)

# Set gripper only
driver.set_gripper_position(goal_position=0.05)  # meters
```

**Cartesian Space:**
```python
# Get current Cartesian position
cart_pos = driver.get_cartesian_positions()  # [x, y, z, rx, ry, rz]

# Set Cartesian position
driver.set_cartesian_positions(
    goal_positions=[0.3, 0.0, 0.4, 0.0, 0.0, 0.0],      # [m, m, m, rad, rad, rad]
    interpolation_space=trossen_arm.InterpolationSpace.cartesian,
    goal_time=2.0,
    blocking=True,
    goal_feedforward_velocities=None,
    goal_feedforward_accelerations=None,
    num_trajectory_check_samples=1000  # collision/limit checking
)
```

**Notes on Cartesian Representation:**
- **Position**: First 3 = translation (x, y, z), Last 3 = **angle-axis rotation** (not Euler angles!)
- **Velocity**: First 3 = linear velocity, Last 3 = angular velocity
- **Acceleration**: First 3 = linear acceleration, Last 3 = angular acceleration

### **C. Velocity Control**

```python
# Joint space velocity
driver.set_all_velocities(
    goal_velocities=[0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0],  # rad/s or m/s
    goal_time=2.0,
    blocking=True,
    goal_feedforward_accelerations=None
)

# Cartesian space velocity
driver.set_cartesian_velocities(
    goal_velocities=[0.05, 0.0, 0.0, 0.0, 0.0, 0.2],  # m/s and rad/s
    interpolation_space=trossen_arm.InterpolationSpace.joint
)
```

### **D. Force/Torque Control**

**External Effort (Gravity Compensation):**
```python
# Zero external effort = gravity compensation
driver.set_all_external_efforts(
    goal_external_efforts=[0, 0, 0, 0, 0, 0, 0],  # Nm or N
    goal_time=0.0,
    blocking=False
)

# Cartesian external effort
driver.set_cartesian_external_efforts(
    goal_external_efforts=[10.0, 0, 0, 0, 0, 0],  # Force + Torque
    interpolation_space=trossen_arm.InterpolationSpace.joint
)
```

**Direct Effort:**
```python
driver.set_all_efforts(
    goal_efforts=[0.5, 0.5, 0.2, 0.0, 0.0, 0.0, 0.0],  # Nm or N
    goal_time=2.0,
    blocking=True
)
```

### **E. Mode Management**

```python
# Set mode for all joints
driver.set_all_modes(trossen_arm.Mode.position)

# Set mode for arm only (excludes gripper)
driver.set_arm_modes(trossen_arm.Mode.external_effort)

# Set mode for gripper only
driver.set_gripper_mode(trossen_arm.Mode.position)

# Set individual joint modes
driver.set_joint_modes([Mode.position, Mode.position, Mode.idle, ...])

# Get current modes
modes = driver.get_modes()  # Returns list[Mode]
```

---

## **6. Sensor Feedback & State Reading**

### **Joint State**
```python
# Position
positions = driver.get_all_positions()          # All joints
arm_pos = driver.get_arm_positions()            # Arm only
gripper_pos = driver.get_gripper_position()     # Gripper only
joint_pos = driver.get_joint_position(0)        # Single joint

# Velocity
velocities = driver.get_all_velocities()

# Acceleration
accelerations = driver.get_all_accelerations()

# Effort (torque/force)
efforts = driver.get_all_efforts()

# External efforts (measured forces)
external_efforts = driver.get_all_external_efforts()

# Compensation efforts (gravity/friction compensation)
comp_efforts = driver.get_all_compensation_efforts()

# Temperature
driver_temps = driver.get_all_driver_temperatures()    # °C
rotor_temps = driver.get_all_rotor_temperatures()      # °C
```

### **Cartesian State**
```python
cart_positions = driver.get_cartesian_positions()          # [x, y, z, rx, ry, rz]
cart_velocities = driver.get_cartesian_velocities()        # [vx, vy, vz, wx, wy, wz]
cart_accelerations = driver.get_cartesian_accelerations()  # [ax, ay, az, αx, αy, αz]
cart_efforts = driver.get_cartesian_external_efforts()     # [fx, fy, fz, τx, τy, τz]
```

### **Complete Robot Output**
```python
output = driver.get_robot_output()  # RobotOutput object

# Header
output.header.id                    # Consecutive ID since config
output.header.timestamp             # Microseconds since config

# Joint data
output.joint.all.positions
output.joint.all.velocities
output.joint.all.efforts
output.joint.all.external_efforts
output.joint.all.accelerations
output.joint.all.compensation_efforts
output.joint.all.driver_temperatures
output.joint.all.rotor_temperatures

# Cartesian data
output.cartesian.positions
output.cartesian.velocities
output.cartesian.accelerations
output.cartesian.external_efforts
```

---

## **7. Advanced Configuration**

### **Joint Limits** (`JointLimit` Class)
```python
limits = driver.get_joint_limits()  # List[JointLimit]

# Modify limits
for limit in limits:
    limit.position_min = -π          # rad or m
    limit.position_max = π
    limit.velocity_max = 2.0         # rad/s or m/s
    limit.effort_max = 10.0          # Nm or N
    limit.position_tolerance = 0.01  # rad or m
    limit.velocity_tolerance = 0.05
    limit.effort_tolerance = 0.5

driver.set_joint_limits(limits)
```

### **Joint Characteristics** (`JointCharacteristic` Class)
```python
chars = driver.get_joint_characteristics()

for char in chars:
    char.position_offset = 0.0                    # rad or m
    char.effort_correction = 1.0                  # Range: [0.2, 5.0]
    char.friction_constant_term = 0.1             # Nm or N
    char.friction_coulomb_coef = 0.5              # Dimensionless
    char.friction_viscous_coef = 0.01             # Nm/(rad/s) or N/(m/s)
    char.friction_transition_velocity = 0.05      # Must be positive

driver.set_joint_characteristics(chars)
```

### **Motor Parameters** (`MotorParameter` Class)
```python
motor_params = driver.get_motor_parameters()  # List[Dict[Mode, MotorParameter]]

# Access PID parameters for a specific mode
position_pid = motor_params[0][trossen_arm.Mode.position].position

position_pid.kp = 100.0      # Proportional gain
position_pid.ki = 0.5        # Integral gain
position_pid.kd = 5.0        # Derivative gain
position_pid.imax = 10.0     # Integral windup limit

driver.set_motor_parameters(motor_params)
```

### **Algorithm Parameters** (`AlgorithmParameter`)
```python
algo = driver.get_algorithm_parameter()
algo.singularity_threshold = 0.01  # Threshold for singularity avoidance
driver.set_algorithm_parameter(algo)
```

### **Network Configuration**
```python
# IP Method
driver.set_ip_method(trossen_arm.IPMethod.manual)  # or IPMethod.dhcp
driver.set_manual_ip("192.168.1.10")
driver.set_gateway("192.168.1.1")
driver.set_subnet("255.255.255.0")
driver.set_dns("8.8.8.8")

# Get network info
ip = driver.get_manual_ip()
gateway = driver.get_gateway()
```

### **YAML Configuration**
```python
# Save all configs to file
driver.save_configs_to_file("/path/to/config.yaml")

# Load configs from file
driver.load_configs_from_file("/path/to/config.yaml")
```

### **Factory Reset**
```python
# Set flag to reset on next startup
driver.set_factory_reset_flag(True)
flag = driver.get_factory_reset_flag()  # Check flag
```

---

## **8. Error Handling**

```python
# Get error information
error_info = driver.get_error_information()  # Returns string

# Get version info
controller_version = driver.get_controller_version()
driver_version = driver.get_driver_version()

# Get number of joints
num_joints = driver.get_num_joints()  # Returns 7 for WXAI V0
```

**Custom Exceptions:**
- **`trossen_arm.LogicError`** - Inherits from `AssertionError`
- **`trossen_arm.RuntimeError`** - Inherits from Python's `RuntimeError`

---

## **9. Static Utility Methods**

```python
# Check IP reachability
is_reachable = TrossenArmDriver.check_ip_reachability(
    serv_ip="192.168.1.2",
    timeout=5.0
)

# Check compatibility (probably for firmware/driver versions)
is_compatible = TrossenArmDriver.check_compatibility(...)
```

---

## **10. Data Type Classes**

### **ArrayDouble3, ArrayDouble6, ArrayDouble9**
Fixed-size arrays for 3D, 6D, and 9D vectors (like numpy arrays but fixed size)

### **VectorDouble**
Dynamic-size list of doubles (like Python list of floats)

### **Link** (Robot link properties)
```python
link.mass                # kg
link.inertia            # 3x3 inertia matrix
link.origin_xyz         # [x, y, z]
link.origin_rpy         # [roll, pitch, yaw]
```

---

## **11. Key Usage Patterns**

### **Blocking vs Non-Blocking**
```python
# Blocking (waits for completion)
driver.set_all_positions(positions, 2.0, blocking=True)

# Non-blocking (returns immediately, continues in background)
driver.set_all_positions(positions, 2.0, blocking=False)
```

### **Feedforward Control**
Improves tracking accuracy for dynamic motions:
```python
driver.set_all_positions(
    positions,
    goal_time=0.01,  # Fast update rate
    blocking=False,
    goal_feedforward_velocities=desired_velocities,
    goal_feedforward_accelerations=desired_accelerations
)
```

This comprehensive API provides full control over Trossen robotic arms from high-level Cartesian commands to low-level motor parameters!