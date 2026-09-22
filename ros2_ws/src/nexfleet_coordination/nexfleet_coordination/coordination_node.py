import rclpy
from rclpy.node import Node
from nexfleet_msgs.msg import AMRState, RiskAssessment
import json

class CoordinationNode(Node):
    def __init__(self):
        super().__init__('coordination_node')
        self.get_logger().info('Starting Federated Coordination Node')
        
        self.state_sub = self.create_subscription(
            AMRState,
            '/nexfleet/amr_states',
            self.state_callback,
            10
        )
        self.risk_pub = self.create_publisher(RiskAssessment, '/nexfleet/risk_assessments', 10)
        self.states = {}

    def state_callback(self, msg: AMRState):
        self.states[msg.robot_id] = msg
        self.evaluate_risks()

    def evaluate_risks(self):
        # Stub for GNN integration on ROS2 side
        for r1, s1 in self.states.items():
            for r2, s2 in self.states.items():
                if r1 >= r2:
                    continue
                dist = abs(s1.row - s2.row) + abs(s1.col - s2.col)
                if dist < 3:
                    risk = RiskAssessment()
                    risk.header.stamp = self.get_clock().now().to_msg()
                    risk.robot_id = r1
                    risk.target_robot_id = r2
                    risk.collision_risk = 0.8
                    risk.risk_level = 'HIGH'
                    risk.recommendation = 'YIELD'
                    self.risk_pub.publish(risk)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
