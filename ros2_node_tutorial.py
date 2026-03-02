#!/usr/bin/env python3
"""
ROS2 node tutorial (code-level).

A ROS2 node is a process that participates in the ROS2 graph: it can
publish messages, subscribe to topics, call/provide services, use actions,
and have parameters. Below we show the minimal pieces and how they fit.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# -----------------------------------------------------------------------------
# 1. What is a node?
# -----------------------------------------------------------------------------
# A node has:
#   - a name (string)
#   - optional namespace
#   - the ability to create publishers, subscribers, services, action clients/servers
#   - callbacks that run when messages (or service requests) arrive
#
# Nodes do not run by themselves. They must be added to an "executor", and
# you call executor.spin() (or spin_until_future) so that the executor
# invokes callbacks when data arrives.


# -----------------------------------------------------------------------------
# 2. Option A: Create a node with rclpy.create_node (minimal, no class)
# -----------------------------------------------------------------------------
def minimal_node_example():
    node = rclpy.create_node("my_minimal_node", namespace="")
    # Now you can: node.create_publisher(...), node.create_subscription(...), etc.
    node.get_logger().info("Minimal node created")
    node.destroy_node()


# -----------------------------------------------------------------------------
# 3. Option B: Subclass Node (usual style in real packages)
# -----------------------------------------------------------------------------
class MyNode(Node):
    """
    A node that subscribes to a topic and publishes on another.
    This is the typical pattern: one class = one node, setup in __init__.
    """

    def __init__(self):
        # Node name and optional namespace
        super().__init__("my_tutorial_node")

        # --- Publisher: send messages to a topic
        self.pub = self.create_publisher(String, "chatter", 10)

        # --- Subscriber: when a message arrives on "in_topic", self._callback is called
        self.sub = self.create_subscription(String, "in_topic", self._callback, 10)

        self.get_logger().info("MyNode: publisher and subscriber created")

    def _callback(self, msg):
        """This runs whenever a message is received on 'in_topic' (e.g. from another node)."""
        self.get_logger().info(f"Received: {msg.data}")
        out = String()
        out.data = f"Echo: {msg.data}"
        self.pub.publish(out)


# -----------------------------------------------------------------------------
# 4. Running the node: executor and spin
# -----------------------------------------------------------------------------
def run_node_example():
    rclpy.init()
    node = MyNode()

    # Executor: it owns one or more nodes and runs their callbacks
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # spin: block and process incoming messages (calls _callback when messages arrive)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


# -----------------------------------------------------------------------------
# 5. Summary: what a node "includes" (code-level)
# -----------------------------------------------------------------------------
# - Identity: name, namespace (from Node.__init__ or create_node).
# - Publishers: create_publisher(msg_type, topic_name, qos_depth).
# - Subscribers: create_subscription(msg_type, topic_name, callback, qos_depth).
# - Services: create_service / create_client (same idea: type, name, callback).
# - Actions: create_action_client / create_action_server (for long-running tasks).
# - Parameters: declare_parameter, get_parameter.
# - Logger: get_logger().info(...), .warn(...), .error(...).
#
# The node does not "run" by itself. You must add it to an executor and
# call spin() so that when data arrives, the executor invokes your callbacks.
# In our mission_scenario.py:
#   - DroneInterface is a Node (from as2_python_api); it has its own executor and spin thread.
#   - aruco_detector is another node (rclpy.create_node) with a separate executor in a daemon thread.


if __name__ == "__main__":
    run_node_example()
