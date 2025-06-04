import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class GestureToCmdNode(Node):
    def __init__(self):
        super().__init__('gesture_to_cmd')

        self.subscription = self.create_subscription(
            Int32,
            '/predicted_gesture',
            self.gesture_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def gesture_callback(self, msg):
        gesture_id = msg.data
        self.get_logger().info(f'Received gesture ID: {gesture_id}')

        twist = Twist()

        if gesture_id == 0:  # fist
            self.get_logger().info('Gesture: Idle – stopping.')
            twist = self.stop()
        elif gesture_id == 2:  # flexion
            self.get_logger().info('Gesture: Flexion – turning left.')
            twist = self.turn_left()
        elif gesture_id == 3:  # extension
            self.get_logger().info('Gesture: Extension – turning right.')
            twist = self.turn_right()
        elif gesture_id == 1:  # pinch thumb-index
            self.get_logger().info('Gesture: Fist-Index – moving forward.')
            twist = self.move_forward()
        else:
            self.get_logger().info('Gesture ignored (unsupported pinch).')
            return  # nie publikuj nic, ignoruj

        self.publisher.publish(twist)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.2
        return twist

    def turn_left(self):
        twist = Twist()
        twist.angular.z = 0.5
        return twist

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -0.5
        return twist

    def stop(self):
        return Twist()  # domyślnie zeruje ruch

def main(args=None):
    rclpy.init(args=args)
    node = GestureToCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
