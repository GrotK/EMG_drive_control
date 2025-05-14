import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time


class EMGPublisherNode(Node):
    def __init__(self):
        super().__init__('emg_publisher')

        self.publisher = self.create_publisher(Float32MultiArray, '/EMG_signal', 10)

        self.data = np.load('repeats_short_X.npz')['arr_0']

        self.index = 0

        self.timer = self.create_timer(0.5, self.publish_sample)

    def publish_sample(self):
        if self.index < len(self.data):
            sample = self.data[self.index]
            msg = Float32MultiArray()
            msg.data = sample.tolist()  
            self.publisher.publish(msg)
            self.get_logger().info(f'Published EMG sample {self.index}')
            
            self.index += 1
        else:
            self.get_logger().info('All samples published.')
            self.destroy_node()  


def main(args=None):
    rclpy.init(args=args)
    node = EMGPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
