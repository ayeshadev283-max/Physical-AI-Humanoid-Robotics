# Unity Robotics Scene Setup Guide

This guide walks through setting up a Unity scene for robot simulation with ROS 2 integration.

## Prerequisites

- Unity Hub: https://unity.com/download
- Unity Editor 2021.3 LTS or newer
- ROS 2 Humble installed (for ROS-TCP-Endpoint)

## Step 1: Create Unity Project

1. Open Unity Hub → New Project
2. Template: **3D (HDRP)** for photorealistic rendering or **3D** for standard
3. Name: `RobotSimulation`
4. Create Project

## Step 2: Install Unity Robotics Packages

### Via Package Manager

1. Window → Package Manager
2. Click `+` → Add package by git URL
3. Add these packages:

```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

4. Wait for installation to complete

## Step 3: Configure ROS Settings

1. Robotics → ROS Settings
2. Configure:
   - **ROS IP Address**: `127.0.0.1` (localhost) or remote IP
   - **ROS Port**: `10000`
   - **Protocol**: ROS 2
3. Click **Update Settings**

## Step 4: Import URDF

1. Assets → Import Robot from URDF
2. Select your robot's `.urdf` file
3. Import Settings:
   - **Axis Convention**: Y-up (Unity default)
   - **Mesh Decomposition**: VHACD (for accurate collision)
   - **Convex Method**: Convex (faster) or VHACD (more accurate)
4. Click **Import**

**Result**: Robot appears in Hierarchy with ArticulationBody components

## Step 5: Setup Scene Environment

### Ground Plane

1. GameObject → 3D Object → Plane
2. Transform:
   - Position: (0, 0, 0)
   - Scale: (10, 1, 10) for 100m² floor
3. Add Component → Box Collider (for physics)
4. Material: Create PBR material with concrete texture

### Lighting

1. GameObject → Light → Directional Light (sun)
2. Transform → Rotation: (50, -30, 0) for realistic shadows
3. Light Component:
   - Intensity: 1.0
   - Cast Shadows: On
   - Color: Slight yellow tint (255, 250, 240)

### Camera

1. Main Camera → Transform:
   - Position: (-5, 3, -5)
   - Rotation: (20, 45, 0) to view robot
2. Add Component → Camera Controller Script (for orbit camera)

```csharp
// OrbitCamera.cs
using UnityEngine;

public class OrbitCamera : MonoBehaviour
{
    public Transform target;
    public float distance = 5f;
    public float rotationSpeed = 100f;

    void Update()
    {
        if (Input.GetMouseButton(1))
        {
            float h = Input.GetAxis("Mouse X") * rotationSpeed * Time.deltaTime;
            float v = Input.GetAxis("Mouse Y") * rotationSpeed * Time.deltaTime;
            transform.RotateAround(target.position, Vector3.up, h);
            transform.RotateAround(target.position, transform.right, -v);
        }
        transform.LookAt(target);
    }
}
```

## Step 6: Setup ROS 2 Communication

### Robot Controller Script

Create `Assets/Scripts/RobotController.cs`:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;
    public float wheelRadius = 0.1f;
    public float wheelSeparation = 0.4f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("cmd_vel", OnCmdVel);
        ros.RegisterPublisher<TwistMsg>("odom");

        InvokeRepeating("PublishOdometry", 0f, 0.1f);  // 10 Hz
    }

    void OnCmdVel(TwistMsg msg)
    {
        float linear = (float)msg.linear.x;
        float angular = (float)msg.angular.z;

        // Differential drive kinematics
        float leftVel = (linear - angular * wheelSeparation / 2) / wheelRadius;
        float rightVel = (linear + angular * wheelSeparation / 2) / wheelRadius;

        SetWheelVelocity(leftWheel, leftVel);
        SetWheelVelocity(rightWheel, rightVel);
    }

    void SetWheelVelocity(ArticulationBody wheel, float velocity)
    {
        var drive = wheel.xDrive;
        drive.targetVelocity = velocity * Mathf.Rad2Deg;  // Unity uses degrees/s
        wheel.xDrive = drive;
    }

    void PublishOdometry()
    {
        var odom = new TwistMsg();
        // Calculate from wheel velocities
        float leftVel = leftWheel.xDrive.targetVelocity * Mathf.Deg2Rad * wheelRadius;
        float rightVel = rightWheel.xDrive.targetVelocity * Mathf.Deg2Rad * wheelRadius;

        float linear = (leftVel + rightVel) / 2;
        float angular = (rightVel - leftVel) / wheelSeparation;

        odom.linear = new Vector3Msg { x = linear };
        odom.angular = new Vector3Msg { z = angular };

        ros.Publish("odom", odom);
    }
}
```

### Attach Script to Robot

1. Select robot root in Hierarchy
2. Add Component → Robot Controller
3. Drag wheel ArticulationBodies to fields in Inspector

## Step 7: Start ROS 2 Endpoint

In terminal (Linux/Mac) or WSL (Windows):

```bash
source /opt/ros/humble/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## Step 8: Test

### In Unity

1. Press Play
2. Check Console for "Connected to ROS" message

### In ROS 2 Terminal

```bash
# Publish velocity command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"

# Monitor odometry
ros2 topic echo /odom
```

**Expected**: Robot moves forward and turns

## Advanced: Add Sensors

### Camera Sensor

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public Camera cam;
    public string topicName = "camera/image";
    public int publishRate = 10;  // Hz

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
        InvokeRepeating("PublishImage", 0f, 1f / publishRate);
    }

    void PublishImage()
    {
        RenderTexture rt = new RenderTexture(640, 480, 24);
        cam.targetTexture = rt;
        cam.Render();

        Texture2D tex = new Texture2D(640, 480, TextureFormat.RGB24, false);
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        tex.Apply();

        var msg = new ImageMsg
        {
            height = 480,
            width = 640,
            encoding = "rgb8",
            step = 640 * 3,
            data = tex.GetRawTextureData()
        };

        ros.Publish(topicName, msg);
    }
}
```

## Troubleshooting

**Robot doesn't respond to /cmd_vel**:
- Check ROS-TCP-Endpoint is running
- Verify IP address in ROS Settings
- Check Console for connection errors

**ArticulationBody jitters**:
- Increase solver iterations: Edit → Project Settings → Physics → Default Solver Iterations = 12
- Reduce stiffness in ArticulationBody drives

**Poor performance**:
- Reduce shadow quality: Edit → Project Settings → Quality
- Disable post-processing if not needed
- Use standard render pipeline instead of HDRP

## Next Steps

- Add LiDAR sensor (raycast-based)
- Implement SLAM visualization
- Add domain randomization for RL
- Export trained policy from Python to Unity (ONNX)
