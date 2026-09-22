"""ROS2/DDS transport adapter for NEXFLEET.

This module provides the ROS2Transport class that implements the same
Transport interface as InProcessTransport. It requires rclpy to be
available (i.e., running inside a ROS2 environment, typically Ubuntu WSL).

On Windows without ROS2, this module is importable but will raise
ROS2UnavailableError if instantiated.
"""
from __future__ import annotations
from typing import Any, Callable, Optional
from backend.communication.transport import Message, NetworkConfig, Transport


class ROS2UnavailableError(RuntimeError):
    """Raised when rclpy is not available."""
    pass


class ROS2Transport(Transport):
    """
    ROS2/DDS transport implementation.

    Architecture:
        - Each NEXFLEET component creates one ROS2Transport.
        - Topics are mapped to ROS2 std_msgs/String topics.
        - QoS: BEST_EFFORT for high-frequency state, RELIABLE for tasks/failures.
        - Requires rclpy and a live ROS2 daemon.
    """

    RELIABLE_TOPICS = {
        "nexfleet/tasks",
        "nexfleet/failure",
        "nexfleet/auction",
        "nexfleet/coordination",
    }

    def __init__(self, node_name: str = "nexfleet_node"):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        except ImportError:
            raise ROS2UnavailableError(
                "rclpy not available. Run in a ROS2 environment (Ubuntu WSL). "
                "Using InProcessTransport instead."
            )

        if not rclpy.ok():
            rclpy.init()

        self._node = Node(node_name)
        self._publishers = {}
        self._subscriptions = {}
        self._callbacks = {}
        self._message_count = 0
        self._rclpy = rclpy
        self._node_name = node_name

        try:
            from std_msgs.msg import String
            self._msg_type = String
        except ImportError:
            raise ROS2UnavailableError("std_msgs not available")

    def _get_qos(self, topic: str):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        if topic in self.RELIABLE_TOPICS:
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            )
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5,
        )

    def _ros_topic(self, topic: str) -> str:
        return "/" + topic.replace("/", "_").replace("-", "_")

    async def publish(self, topic: str, sender: str, payload: Any) -> bool:
        import json
        ros_topic = self._ros_topic(topic)
        if ros_topic not in self._publishers:
            self._publishers[ros_topic] = self._node.create_publisher(
                self._msg_type, ros_topic, self._get_qos(topic)
            )
        msg = self._msg_type()
        msg.data = json.dumps({"sender": sender, "payload": payload})
        self._publishers[ros_topic].publish(msg)
        self._message_count += 1
        return True

    def subscribe(self, topic: str, callback: Callable[[Message], None]):
        import json
        ros_topic = self._ros_topic(topic)

        def ros_callback(msg):
            try:
                data = json.loads(msg.data)
                m = Message(
                    topic=topic,
                    sender=data.get("sender", "unknown"),
                    payload=data.get("payload"),
                )
                callback(m)
            except Exception as e:
                print(f"[ROS2Transport] Callback error: {e}")

        sub = self._node.create_subscription(
            self._msg_type, ros_topic, ros_callback, self._get_qos(topic)
        )
        if ros_topic not in self._subscriptions:
            self._subscriptions[ros_topic] = []
        self._subscriptions[ros_topic].append(sub)

    async def broadcast(self, sender: str, payload: Any):
        await self.publish("nexfleet/fleet/broadcast", sender, payload)

    def spin_once(self):
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def get_transport_name(self) -> str:
        return "ROS2/DDS"

    def get_message_count(self) -> int:
        return self._message_count

    def destroy(self):
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
