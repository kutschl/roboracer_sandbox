import os
os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '1'
os.environ['RCUTILS_LOGGING_BUFFERED_STREAM'] = '1'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import ListParameters, GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from functools import partial
from PyQt5.QtCore import QTimer, QThread

class ROS2Thread(QThread):
    """Spin the ROS 2 node in its own Qt thread so the GUI stays responsive."""

    def __init__(self, ros_node: Node):
        super().__init__()
        self._ros_node = ros_node

    def run(self):
        rclpy.spin(self._ros_node)

class SubscriberNode(Node):
    def __init__(self):
        rclpy.init()
        # Initialize rclpy if needed (rclpy.init() is typically done externally)
        super().__init__(
            'sub_node', 
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True
        )
        # ROS2 Publisher/Subscriber attributes
        self._impl_subscribers = {}
        self._sub_data = {}
        self._impl_publishers = {}
        # Add attributes for parameter services
        self._current_node = ''
        self._impl_clients = {}

    # ====================
    # Discovery helpers
    # ====================
    def query_nodes(self):
        """Return the list of fully qualified node names currently in the graph."""
        # get_node_names_and_namespaces() returns a list of tuples (name, namespace)
        # Adjust if necessary for your ROS 2 version.
        return [
            f"{ns.rstrip('/')}/{name}" if ns != '/' else name
            for name, ns in self.get_node_names_and_namespaces()
        ]

    def set_node_of_interest(self, node_name: str):
        """Set the target node for parameter operations."""
        self._current_node = node_name
        self._impl_clients.clear()  # Clear previously created clients

    # ====================
    # Client helpers
    # ====================
    def _get_client(self, svc_type, suffix):
        """Create (or retrieve) a client for self._current_node/suffix."""
        key = (svc_type, suffix)
        if key not in self._impl_clients:
            self._impl_clients[key] = self.create_client(
                svc_type, f"{self._current_node}/{suffix}"
            )
        return self._impl_clients[key]

    # ====================
    # Parameter operations
    # ====================
    async def list_parameters(self):
        """List all parameters for the current node."""
        client = self._get_client(ListParameters, 'list_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('list_parameters service not available')
            return []
        
        # Use recursive depth to get all parameters
        req = ListParameters.Request(depth=ListParameters.Request.DEPTH_RECURSIVE)
        resp = await client.call_async(req)
        # Names reside within the resp.result
        return resp.result.names if resp is not None else []

    async def get_parameter_values(self, names):
        """Retrieve parameter values for a list of parameter names."""
        client = self._get_client(GetParameters, 'get_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('get_parameters service not available')
            return []
        
        req = GetParameters.Request(names=names)
        resp = await client.call_async(req)
        return resp.values if resp is not None else []

    async def set_parameter(self, name, value):
        """Set the parameter with the given name and value."""
        client = self._get_client(SetParameters, 'set_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('set_parameters service not available')
            return
        
        param_msg = Parameter(name=name.lstrip('/'))
        param_msg.value = ParameterValue()

        if isinstance(value, bool):
            param_msg.value.type = ParameterType.PARAMETER_BOOL
            param_msg.value.bool_value = value
        elif isinstance(value, int):
            param_msg.value.type = ParameterType.PARAMETER_INTEGER
            param_msg.value.integer_value = value
        elif isinstance(value, float):
            param_msg.value.type = ParameterType.PARAMETER_DOUBLE
            param_msg.value.double_value = value
        elif isinstance(value, str):
            param_msg.value.type = ParameterType.PARAMETER_STRING
            param_msg.value.string_value = value
        else:
            self.get_logger().warning(f'Unsupported parameter type for {name!r}')
            return

        req = SetParameters.Request(parameters=[param_msg])
        await client.call_async(req)

    # ====================
    # Existing Subscription/Publisher functionalities (unchanged)
    # ====================
    def __call__(self, topic: str):
        if topic not in self._sub_data:
            return None
        return self._sub_data[topic]
    
    def publish(self, topic: str, msg):
        if topic not in self._impl_publishers:
            raise ValueError(f"Publisher for topic {topic} does not exist.")
        
        pub = self._impl_publishers[topic]
        pub.publish(msg)
    
    def subscribe(self, topic: str, msg_type):
        if topic in self._impl_subscribers:
            return self._impl_subscribers[topic]
        
        sub = self.create_subscription(
            msg_type, topic, partial(self.callback, topic), qos_profile_sensor_data
        )
        self._impl_subscribers[topic] = sub
        self._sub_data[topic] = None

    def unsubscribe(self, topic: str):
        if topic not in self._impl_subscribers:
            return

        self.destroy_subscription(self._impl_subscribers[topic])
        del self._impl_subscribers[topic]
        del self._sub_data[topic]
    
    def publisher(self, topic: str, msg_type):
        pub = self.create_publisher(msg_type, topic, 5)
        self._impl_publishers[topic] = pub

    def delete_publisher(self, topic: str):
        pub = self._impl_publishers.get(topic)
        if pub is not None:
            self.destroy_publisher(pub)
            del self._impl_publishers[topic]
        else:
            raise ValueError(f"Publisher for topic {topic} does not exist.")
              
    def callback(self, topic: str, msg):
        self._sub_data[topic] = msg

    def destroy(self):
        # Clean up subscriptions and publishers
        for topic in list(self._impl_subscribers.keys()):
            self.unsubscribe(topic)
        for topic in list(self._impl_publishers.keys()):
            self.delete_publisher(topic)
        self.destroy_node()
        rclpy.shutdown()