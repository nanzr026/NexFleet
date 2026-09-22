import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nexfleet_msgs.msg import AMRState, TaskCommand
import json

class AMRDDSNode(Node):
    def __init__(self):
        super().__init__('amr_dds_node')
        self.declare_parameter('robot_id', 'AMR-01')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        
        self.get_logger().info(f'Starting AMR DDS Node for {self.robot_id}')
        
        # Publishers
        self.state_pub = self.create_publisher(AMRState, '/nexfleet/amr_states', 10)
        
        # Subscribers
        self.task_sub = self.create_subscription(
            TaskCommand, 
            f'/nexfleet/{self.robot_id}/task_command',
            self.task_callback,
            10
        )
        
        # Simulate local control loop
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.mock_col = 1
        self.mock_row = 1
        
    def task_callback(self, msg: TaskCommand):
        self.get_logger().info(f'Received task: {msg.task_id} -> ({msg.target_row}, {msg.target_col})')
        
    def timer_callback(self):
        msg = AMRState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.row = self.mock_row
        msg.col = self.mock_col
        msg.status = 'IDLE'
        msg.battery = 98.5
        
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AMRDDSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
