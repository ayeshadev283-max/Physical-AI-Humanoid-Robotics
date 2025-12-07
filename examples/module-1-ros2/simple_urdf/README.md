# Simple URDF Example

This example demonstrates a basic mobile robot URDF with:
- Parameterized base link
- Reusable wheel macro (4 wheels)
- Caster wheel for stability
- Proper inertial properties

## Build URDF

```bash
xacro robot.urdf.xacro > robot.urdf
```

## Visualize

```bash
ros2 launch urdf_tutorial display.launch.py model:=robot.urdf
```

## Key Concepts

- **Xacro properties**: `base_width`, `wheel_radius` for easy tuning
- **Macros**: `wheel` macro avoids repeating wheel definition 4 times
- **Joint types**: `continuous` for wheels, `fixed` for caster
- **Inertial properties**: Required for physics simulation
