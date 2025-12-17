---
title: "ROS 2 with Python - rclpy Usage"
description: "Comprehensive guide to using ROS 2 with Python through the rclpy client library"
tags: ["ROS 2", "Python", "rclpy", "Robotics", "Programming"]
---

# ROS 2 with Python - rclpy Usage

## Learning Objectives

By the end of this chapter, students will be able to:
- Create ROS 2 nodes using rclpy
- Implement publishers, subscribers, services, and actions in Python
- Handle parameters and logging in ROS 2 Python nodes
- Design effective node architectures using rclpy
- Debug and optimize Python-based ROS 2 applications

## Introduction to rclpy

rclpy is the Python client library for ROS 2, providing a Python API to interact with the ROS 2 middleware. It allows Python developers to create ROS 2 nodes that can communicate with nodes written in other languages like C++.

rclpy provides:
- Node creation and management
- Publisher and subscriber functionality
- Service and action clients and servers
- Parameter handling
- Logging capabilities
- Time and timer utilities

## Setting Up a Basic Node

### Creating Your First Node

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node created')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Understanding Node Creation

The basic structure consists of:
1. Inheriting from `rclpy.node.Node`
2. Calling `super().__init__()` with a node name
3. Using `rclpy.init()` to initialize the ROS 2 client library
4. Using `rclpy.spin()` to keep the node alive
5. Properly cleaning up with `node.destroy_node()` and `rclpy.shutdown()`

## Publishers and Subscribers in rclpy

### Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Creating a Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

### Advanced Publisher Features

#### Publisher with Custom QoS

```python
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class PublisherWithQoS(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        self.publisher_ = self.create_publisher(
            String, 
            'qos_topic', 
            qos_profile
        )
```

#### Publisher with Latching

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class LatchedPublisher(Node):
    def __init__(self):
        super().__init__('latched_publisher')
        
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.publisher_ = self.create_publisher(
            String, 
            'latched_topic', 
            latched_qos
        )
```

## Services in rclpy

### Creating a Service Server

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Creating a Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

### Synchronous Service Client

```python
class MinimalClientSync(Node):
    def __init__(self):
        super().__init__('minimal_client_sync')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

## Actions in rclpy

### Creating an Action Server

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            goal_handle.publish_feedback(feedback_msg)
            
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

### Creating an Action Client

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.sequence))
```

## Parameters in rclpy

### Declaring and Using Parameters

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('param1', 'default_value')
        self.declare_parameter('param2', 10)
        self.declare_parameter('param3', 3.14)
        
        # Get parameter values
        self.param1 = self.get_parameter('param1').value
        self.param2 = self.get_parameter('param2').value
        self.param3 = self.get_parameter('param3').value
        
        self.get_logger().info(f'param1: {self.param1}')
        self.get_logger().info(f'param2: {self.param2}')
        self.get_logger().info(f'param3: {self.param3}')

    def update_parameters_callback(self, parameters):
        for param in parameters:
            if param.name == 'param1' and param.type_ == Parameter.Type.STRING:
                self.param1 = param.value
                self.get_logger().info(f'Updated param1 to: {self.param1}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    
    # Add parameter callback
    node.add_on_set_parameters_callback(node.update_parameters_callback)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Parameter Descriptions

```python
from rcl_interfaces.msg import ParameterDescriptor

class ParameterNodeWithDescriptions(Node):
    def __init__(self):
        super().__init__('param_desc_node')
        
        # Declare parameter with description
        param_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='A string parameter for configuration',
            additional_constraints='Must be a valid configuration option'
        )
        
        self.declare_parameter('config_option', 'default', param_desc)
```

## Logging in rclpy

### Basic Logging

```python
import rclpy
from rclpy.node import Node

class LoggingNode(Node):
    def __init__(self):
        super().__init__('logging_node')
        
        # Different log levels
        self.get_logger().debug('This is a debug message')
        self.get_logger().info('This is an info message')
        self.get_logger().warn('This is a warning message')
        self.get_logger().error('This is an error message')
        self.get_logger().fatal('This is a fatal message')
```

### Logging with Formatting

```python
class AdvancedLoggingNode(Node):
    def __init__(self):
        super().__init__('advanced_logging_node')
        
        # Using formatted logging
        value = 42
        self.get_logger().info(f'The value is {value}')
        
        # Logging with context
        self.get_logger().info('Processing data for robot_name: %s', self.get_parameter('robot_name').value)
```

## Timers in rclpy

### Basic Timer Usage

```python
import rclpy
from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info('Timer callback executed: %d' % self.counter)
        self.counter += 1
```

### Multiple Timers

```python
class MultipleTimerNode(Node):
    def __init__(self):
        super().__init__('multiple_timer_node')
        
        # Different frequency timers
        self.slow_timer = self.create_timer(1.0, self.slow_timer_callback)  # 1 Hz
        self.fast_timer = self.create_timer(0.1, self.fast_timer_callback)  # 10 Hz

    def slow_timer_callback(self):
        self.get_logger().info('Slow timer executed')

    def fast_timer_callback(self):
        self.get_logger().info('Fast timer executed')
```

## Advanced Node Patterns

### Node with Lifecycle Management

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleExampleNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_example_node')
        self.get_logger().info('Lifecycle node created')

    def on_configure(self, state):
        self.get_logger().info('Configuring node')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating node')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating node')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up node')
        return TransitionCallbackReturn.SUCCESS
```

### Node with Multiple Threads

```python
import threading
import rclpy
from rclpy.node import Node

class ThreadedNode(Node):
    def __init__(self):
        super().__init__('threaded_node')
        self.lock = threading.Lock()
        self.shared_data = 0
        
        # Create a timer to simulate periodic work
        self.timer = self.create_timer(0.5, self.main_thread_work)
        
        # Start a background thread
        self.background_thread = threading.Thread(target=self.background_work)
        self.background_thread.start()

    def main_thread_work(self):
        with self.lock:
            self.shared_data += 1
        self.get_logger().info(f'Main thread: shared_data = {self.shared_data}')

    def background_work(self):
        while rclpy.ok():
            with self.lock:
                self.shared_data *= 2
            self.get_logger().info(f'Background thread: shared_data = {self.shared_data}')
            time.sleep(1)
```

## Error Handling

### Exception Handling in Callbacks

```python
import traceback

class ErrorHandlingNode(Node):
    def __init__(self):
        super().__init__('error_handling_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.error_safe_callback,
            10)

    def error_safe_callback(self, msg):
        try:
            # Process message
            result = process_message(msg)
            self.get_logger().info(f'Processed: {result}')
        except Exception as e:
            self.get_logger().error(f'Error processing message: {str(e)}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            
def process_message(msg):
    # Simulate potential error
    if 'error' in msg.data:
        raise ValueError('Error in message')
    return f'Result for: {msg.data}'
```

### Timeout Handling

```python
import rclpy
from rclpy.node import Node
from rclpy.task import Future

class TimeoutNode(Node):
    def __init__(self):
        super().__init__('timeout_node')
        
    def wait_with_timeout(self, future, timeout_sec=5.0):
        """Wait for a future with timeout"""
        timer = self.create_timer(0.1, lambda: None)  # Dummy timer to keep spinning
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.done():
                return future.result()
            else:
                self.get_logger().error('Timeout occurred')
                return None
        finally:
            timer.destroy()
```

## Common Patterns for Robotics Applications

### Sensor Data Processing Pattern

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SensorProcessorNode(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # Store recent sensor data
        self.joint_positions = {}
        
    def joint_state_callback(self, msg):
        # Update internal state
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
        
        # Process data
        self.process_joint_data()
        
    def process_joint_data(self):
        # Perform calculations based on joint positions
        if 'left_leg_joint' in self.joint_positions:
            pos = self.joint_positions['left_leg_joint']
            # Process the position data
            self.get_logger().info(f'Left leg position: {pos}')
```

### Robot Control Pattern

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publishers for control commands
        self.command_publisher = self.create_publisher(
            Float64MultiArray, 
            'joint_commands', 
            10
        )
        
        # Subscribers for sensor feedback
        self.feedback_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.feedback_callback,
            10
        )
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz
        
    def feedback_callback(self, msg):
        # Store feedback for control calculations
        self.current_positions = dict(zip(msg.name, msg.position))
        
    def control_loop(self):
        # Calculate control commands based on desired and current state
        commands = self.calculate_control_command()
        
        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = commands
        self.command_publisher.publish(cmd_msg)
        
    def calculate_control_command(self):
        # Implement your control algorithm here
        return [0.0] * 10  # Placeholder
```

## Performance Optimization Tips

### Efficient Message Handling

```python
# For high-frequency topics, consider using callbacks that do minimal work
class EfficientNode(Node):
    def __init__(self):
        super().__init__('efficient_node')
        # Use smaller queue sizes for time-sensitive data
        self.sub = self.create_subscription(
            String, 'high_freq_topic', 
            self.high_freq_callback, 1)  # Small queue size

    def high_freq_callback(self, msg):
        # Do minimal processing in callback
        # Store data and process it elsewhere if needed
        self.processed_data = msg.data
```

### Memory Management

```python
class MemoryEfficientNode(Node):
    def __init__(self):
        super().__init__('memory_efficient_node')
        # Pre-allocate message objects if publishing frequently
        self.msg_cache = String()  # Reuse this object
        
    def timer_callback(self):
        # Reuse message object instead of creating new ones
        self.msg_cache.data = f'New data at {self.get_clock().now()}'
        self.publisher.publish(self.msg_cache)
```

## Debugging Tips

### Useful rclpy Debugging Techniques

```python
# Check node status
def check_node_status(node):
    print(f"Node name: {node.get_name()}")
    print(f"Node namespace: {node.get_namespace()}")
    print(f"Node clock: {node.get_clock().now()}")
    
    # List all subscriptions and publications
    subscriptions = node.get_subscriptions_info_by_topic('topic_name')
    publishers = node.get_publishers_info_by_topic('topic_name')
    
    print(f"Subscriptions: {subscriptions}")
    print(f"Publishers: {publishers}")
```

## Exercises

1. Create a node that publishes joint commands at 100Hz and another that subscribes to joint states, implementing a simple PD controller.
2. Implement a service that calculates inverse kinematics for a humanoid robot arm.
3. Design an action server that controls a humanoid robot's walking gait with feedback on step progress.
4. Create a parameter server that allows changing robot control parameters at runtime.

## Quiz

1. What is the primary purpose of rclpy?
   - A) To provide a Python interface to ROS 2
   - B) To manage ROS 2 network connections
   - C) To compile ROS 2 code
   - D) To visualize ROS 2 data

2. Which method is used to keep a ROS 2 node active?
   - A) rclpy.run()
   - B) rclpy.spin()
   - C) rclpy.start()
   - D) rclpy.active()

3. How do you properly shut down a ROS 2 node in Python?
   - A) node.shutdown()
   - B) rclpy.stop()
   - C) node.destroy_node() and rclpy.shutdown()
   - D) exit()

## Reflection

Consider how the Python ecosystem and rclpy integration makes ROS 2 accessible to a broader range of developers. What are the trade-offs of using Python for robot control (performance vs. development speed)? How might the design patterns you've learned be applied to complex humanoid robot systems with multiple sensors and actuators? What challenges might arise when using Python in real-time robotic applications?