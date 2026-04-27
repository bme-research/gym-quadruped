# GO2 Robot Cane Modifications

## Summary

The GO2 quadruped robot model has been enhanced with an L-shaped cane structure attached to the back of the robot. This cane includes force-torque sensors that allow the robot to sense external forces applied to the cane handle and adjust its equilibrium accordingly.

## Changes Made

### 1. MuJoCo XML Model (`go2.xml`)

#### Cane Structure
- **Position**: Attached at `(-0.25, 0, 0)` relative to the robot base (at the back of the robot)
- **Vertical Cylinder** (`cane_base` body):
  - Height: 0.8m (80cm)
  - Radius: 0.015m (3cm diameter)
  - Position in body: extends from z=0 to z=0.8m
  - Mass: 0.5kg
  
- **Horizontal Handle** (`cane_handle` body):
  - Length: 0.5m (50cm) 
  - Radius: 0.015m (3cm diameter)
  - Position: At the top of vertical cylinder (z=0.8m)
  - Orientation: Perpendicular to vertical cylinder (rotated 90 degrees)
  - Mass: 0.3kg

#### Force-Torque Sensors
The following sensors measure forces and torques at the cane-base connection:

```xml
<force name="cane_force" body="cane_base"/>      <!-- 3D force vector in N -->
<torque name="cane_torque" body="cane_base"/>    <!-- 3D torque vector in N⋅m -->
<framepos name="cane_pos" objtype="body" objname="cane_base"/>   <!-- 3D position -->
<framequat name="cane_quat" objtype="body" objname="cane_base"/> <!-- Orientation (quaternion) -->
```

#### Joint Configuration
- **Cane Base Joint** (`cane_base_joint`): Free joint type, allowing 6 DOF
- **Cane Handle Joint** (`cane_handle_joint`): Free joint type, allowing 6 DOF

This configuration allows the cane to move freely while sensors capture all reaction forces and torques.

#### Updated Keyframes
Both keyframes (`home` and `down`) have been updated to include the initial state of the cane joints:
- Cane base position: `(-0.25, 0, 0)` with identity quaternion `(1, 0, 0, 0)`
- Cane handle position: `(0, 0, 0.8)` with identity quaternion `(1, 0, 0, 0)` relative to cane_base

### 2. URDF Model (`go2.urdf`)

Parallel updates were made to the URDF file for ROS compatibility:

- **`cane_base` link**: Defines the vertical cylinder with collision geometry
- **`cane_base_joint`**: Floating joint connecting cane to robot base
- **`cane_handle` link**: Defines the horizontal handle cylinder
- **`cane_handle_joint`**: Floating joint connecting handle to cane base

Note: The URDF uses `floating` joint type (equivalent to MuJoCo's free joint).

### 3. Documentation

- Updated `README.md` with description of cane structure and sensors
- Created this file documenting all modifications

## How to Use the Cane Sensors

### Accessing Force-Torque Data in Simulation

In MuJoCo Python API:

```python
import mujoco

# After loading the model and creating data
model = mujoco.MjModel.from_xml_path('go2.xml')
data = mujoco.MjData(model)

# Run simulation
mujoco.mj_step(model, data)

# Access sensor data
# Find sensor indices
cane_force_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'cane_force')
cane_torque_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'cane_torque')

# Get sensor data (shape depends on sensor type)
forces = data.sensordata[cane_force_id:cane_force_id+3]  # 3D force vector
torques = data.sensordata[cane_torque_id:cane_torque_id+3]  # 3D torque vector
```

### Applying Forces to the Cane

To simulate external forces on the cane handle:

```python
# Find cane_handle body
cane_handle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cane_handle')

# Apply force to the handle
force_vector = [10, 0, -5]  # Force in N (x, y, z)
data.xfrc_applied[cane_handle_id, :3] = force_vector

# You can also apply torque
torque_vector = [0, 0, 0]  # Torque in N⋅m
data.xfrc_applied[cane_handle_id, 3:] = torque_vector
```

## Robot Equilibrium and Force Compensation

With the cane sensor setup:

1. **Force Sensing**: External forces applied to the cane handle are measured by the force-torque sensors
2. **Body Dynamics**: These forces/torques affect the motion of the cane and are transmitted through the base to the robot
3. **Equilibrium Control**: The robot can use the sensed forces to adjust:
   - Leg joint angles for balance compensation
   - Center of mass position
   - Stance stability

This is particularly useful for:
- Balance control research
- Human-robot interaction studies
- External perturbation rejection experiments
- Stability analysis under load

## Specifications

| Property | Value |
|----------|-------|
| Vertical cylinder height | 0.8 m |
| Horizontal handle length | 0.5 m |
| Cylinder radius (both) | 0.015 m (3 cm diameter) |
| Vertical cylinder mass | 0.5 kg |
| Handle mass | 0.3 kg |
| Attachment point (back of robot) | x = -0.25 m |
| Sensors | force, torque, framepos, framequat |

## Notes

- The cane is modeled with simple cylinder geometries for computational efficiency
- The free joints allow unrestricted motion - constrain as needed for your application
- Adjust masses if needed for your specific simulation requirements
- The sensor readings are in MuJoCo's default units (meters, kilograms, seconds, Newtons, Newton-meters)
