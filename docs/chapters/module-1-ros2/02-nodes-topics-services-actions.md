---
sidebar_position: 2
title: "Chapter 2: Nodes, Topics, Services, Actions"
description: ROS 2 communication patterns and node lifecycle
tags: [ros2, nodes, topics, services, actions, lifecycle]
---

# Chapter 2: Nodes, Topics, Services, Actions

## Learning Objectives

1. Implement ROS 2 nodes with proper lifecycle management
2. Design pub/sub systems using topics and message types
3. Implement synchronous services and asynchronous actions

## 2.1 Node Lifecycle Management

### Managed vs Unmanaged Nodes

**Unmanaged Node** (default):
- Starts immediately in active state
- No formal state transitions
- Simple for prototyping

**Managed Node** (LifecycleNode):
- Explicit state machine: Unconfigured → Inactive → Active → Finalized
- Allows controlled startup/shutdown
- Required for production systems (ros2_control, Nav2)

### Lifecycle States

```
┌─────────────┐
│ Unconfigured│
└──────┬──────┘
       │ configure()
       ▼
┌─────────────┐
│  Inactive   │
└──────┬──────┘
       │ activate()
       ▼
┌─────────────┐
│   Active    │  ◄─── Normal operation
└──────┬──────┘
       │ deactivate()
       ▼
┌─────────────┐
│  Inactive   │
└──────┬──────┘
       │ cleanup() / shutdown()
       ▼
┌─────────────┐
│  Finalized  │
└─────────────┘
```

**State Callbacks**:
- `on_configure()`: Load parameters, allocate resources
- `on_activate()`: Start publishing, open connections
- `on_deactivate()`: Pause operation, keep resources
- `on_cleanup()`: Release resources
- `on_shutdown()`: Emergency stop

**Example** (Python):
```python
from rclpy.lifecycle import LifecycleNode, LifecycleState

class CameraNode(LifecycleNode):
    def on_configure(self, state: LifecycleState):
        self.camera = Camera(self.get_parameter('device_id').value)
        self.pub = self.create_lifecycle_publisher(Image, 'camera/image', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.camera.start()
        self.timer = self.create_timer(0.033, self.publish_frame)  # 30 Hz
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        self.timer.cancel()
        self.camera.stop()
        return super().on_deactivate(state)
```

## 2.2 Publish-Subscribe Pattern (Topics)

### Message Types

**Standard Messages** (`ros2 interface list`):
- `std_msgs`: Primitive types (Int32, Float64, String, Bool)
- `geometry_msgs`: Pose, Twist, Transform, Wrench
- `sensor_msgs`: Image, PointCloud2, LaserScan, Imu, JointState
- `nav_msgs`: Odometry, Path, OccupancyGrid

**Custom Messages**:
```
# my_robot_msgs/msg/BatteryStatus.msg
float32 voltage
float32 current
float32 percentage
uint8 STATUS_OK=0
uint8 STATUS_LOW=1
uint8 STATUS_CRITICAL=2
uint8 status
```

Build with `colcon build`, generates Python/C++ classes.

### Publisher Example (C++)

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class VelocityPublisher : public rclcpp::Node {
public:
    VelocityPublisher() : Node("velocity_publisher") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VelocityPublisher::publish_cmd, this));
    }

private:
    void publish_cmd() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.5;  // Forward at 0.5 m/s
        msg.angular.z = 0.1; // Turn at 0.1 rad/s
        pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

### Subscriber Example (Python)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)  # QoS depth

    def scan_callback(self, msg: LaserScan):
        min_distance = min(msg.ranges)
        if min_distance < 0.5:
            self.get_logger().warn(f'Obstacle at {min_distance:.2f}m!')
```

### Design Patterns

**Throttling**:
```python
from rclpy.time import Duration
self.last_publish = self.get_clock().now()

if (self.get_clock().now() - self.last_publish) > Duration(seconds=1.0):
    self.publisher.publish(msg)
    self.last_publish = self.get_clock().now()
```

**Message Filtering**:
```python
from message_filters import Subscriber, TimeSynchronizer

image_sub = Subscriber(self, Image, '/camera/image')
depth_sub = Subscriber(self, Image, '/camera/depth')
sync = TimeSynchronizer([image_sub, depth_sub], queue_size=10)
sync.registerCallback(self.rgbd_callback)
```

## 2.3 Request-Reply Pattern (Services)

### When to Use Services

**Services**: Discrete, short-duration tasks
- Reset odometry
- Capture a single image
- Save map to file
- Query robot state

**Not for**: Long tasks (use actions), streaming (use topics)

### Service Definition

```
# example_interfaces/srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Service Server (Python)

```python
from example_interfaces.srv import AddTwoInts

class AdderService(Node):
    def __init__(self):
        super().__init__('adder')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client (C++)

```cpp
auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
request->a = 5;
request->b = 7;

auto future = client->async_send_request(request);
// Wait for response
if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    RCLCPP_INFO(node->get_logger(), "Sum: %ld", result->sum);
}
```

### Timeout Handling

```python
import rclpy
from rclpy.client import Client

client = node.create_client(Trigger, '/reset_odometry')
if not client.wait_for_service(timeout_sec=5.0):
    node.get_logger().error('Service not available!')
    return

future = client.call_async(Trigger.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
```

## 2.4 Goal-Based Pattern (Actions)

### When to Use Actions

**Actions**: Long-running, preemptable tasks
- Navigate to waypoint (can cancel mid-flight)
- Pick and place object (feedback on grasp progress)
- Trajectory execution (report completion percentage)

**Advantages**:
- Cancellable: Client can abort
- Feedback: Progress updates
- Result: Final outcome

### Action Definition

```
# action/Fibonacci.action
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

### Action Server (Python)

```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciServer(Node):
    def __init__(self):
        super().__init__('fibonacci_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        feedback = Fibonacci.Feedback()
        feedback.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            # Compute next number
            feedback.partial_sequence.append(
                feedback.partial_sequence[-1] + feedback.partial_sequence[-2])
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)  # Simulate work

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback.partial_sequence
        return result
```

### Action Client (C++)

```cpp
auto action_client = rclcpp_action::create_client<Fibonacci>(node, "fibonacci");

auto goal_msg = Fibonacci::Goal();
goal_msg.order = 10;

auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
send_goal_options.feedback_callback = [](auto, const auto & feedback) {
    std::cout << "Feedback: " << feedback->partial_sequence.back() << std::endl;
};
send_goal_options.result_callback = [](const auto & result) {
    std::cout << "Final sequence size: " << result.result->sequence.size() << std::endl;
};

auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
```

### Preemption (Cancellation)

```python
# Client cancels goal
future = action_client.send_goal_async(goal)
goal_handle = future.result()
cancel_future = goal_handle.cancel_goal_async()
```

## Summary

**Lifecycle**: Managed nodes for controlled startup/shutdown
**Topics**: Many-to-many, async, streaming data
**Services**: One-to-one, sync, discrete requests
**Actions**: Goal-feedback-result, async, preemptable



## Exercises

**Exercise 2.1**: Implement a LifecycleNode that controls an LED. The LED should turn on in the `on_activate()` callback and turn off in the `on_deactivate()` callback. Test state transitions using `ros2 lifecycle set /led_node`.

**Exercise 2.2**: Create a service that calculates the distance between two 3D points. Define a custom service type with two `geometry_msgs/Point` as input and a `float64` distance as output. Implement the server in C++ and client in Python.

**Exercise 2.3**: Implement an action server for a simple battery charging simulation. The goal is the target charge percentage, feedback is the current percentage (updated every second), and the result indicates whether charging succeeded or was preempted. Test cancellation mid-charging.

**Exercise 2.4**: Synchronize two camera topics (RGB and depth) using `message_filters.TimeSynchronizer`. Process only pairs of messages with matching timestamps. Log a warning if synchronization fails for more than 1 second.


**Next**: Chapter 3 covers coordinate transforms (TF2) and robot descriptions (URDF).
