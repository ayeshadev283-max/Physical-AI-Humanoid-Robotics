# Gazebo Warehouse World Example

This example demonstrates a Gazebo world file for robot simulation with:
- DART physics engine for accurate contact dynamics
- Warehouse environment (walls, shelves)
- Dynamic manipulable objects (boxes)
- Realistic lighting and materials
- ROS 2 integration via plugins

## Features

**Physics Configuration**:
- Engine: DART (best for manipulation)
- Step size: 0.001s (1ms) for stability
- Real-time factor: 1.0 (run at normal speed)

**Environment**:
- Ground plane with friction (μ=1.0)
- Warehouse walls (20m x 20m)
- Storage shelves (static)
- Manipulable boxes (dynamic, 0.5kg)

**Materials**:
- Boxes: μ=0.8 (realistic friction for cardboard)
- Contact stiffness: kp=1e6 N/m (rigid)
- Contact damping: kd=100 N·s/m (stable)

## Usage

### Launch World

```bash
gz sim warehouse.sdf
```

### With ROS 2 Bridge

```bash
# Terminal 1: Launch Gazebo
gz sim warehouse.sdf

# Terminal 2: Bridge topics
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

### Spawn Robot

```bash
# After launching world
ros2 run ros_gz_sim create -world warehouse -file path/to/robot.urdf -name my_robot -x 0 -y 0 -z 0.5
```

## Customization

**Add More Objects**:
```xml
<model name="box_02">
  <pose>2 2 0.5 0 0 0</pose>
  <!-- Copy box_01 link definition -->
</model>
```

**Change Physics Engine**:
```xml
<physics name="ode_physics" type="ode">
  <!-- ODE configuration -->
</physics>
```

**Adjust Lighting**:
```xml
<light type="point" name="lamp">
  <pose>0 0 5 0 0 0</pose>
  <diffuse>1 1 1 1</diffuse>
  <attenuation>
    <range>20</range>
  </attenuation>
</light>
```

## Key Concepts

- **Static models**: Walls, shelves (don't move)
- **Dynamic models**: Boxes (affected by physics)
- **Surface properties**: Friction, contact stiffness/damping
- **Inertial properties**: Mass, moment of inertia (for dynamics)
- **Plugins**: ROS 2 integration, custom behaviors
