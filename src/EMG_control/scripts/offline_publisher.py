import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time


class EMGPublisherNode(Node):
    def __init__(self):
        super().__init__('emg_publisher')

        self.publisher = self.create_publisher(Float32MultiArray, '/EMG_signal', 10)

        data_path='/workspace/sequential_X/sequential_X.npz'

        self.data = self.flatten_overlapping_windows(np.load(data_path)['arr'])
        self.index = 0

        self.timer = self.create_timer(0.01, self.publish_sample)

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

    def flatten_overlapping_windows(self, data, stride=512):
        num_windows, window_size, num_channels = data.shape
        result = []

        result.append(data[0])

        for i in range(1, num_windows):
            result.append(data[i][-stride:])

        flattened_data = np.concatenate(result, axis=0) 
        print(np.shape(flattened_data))
        return flattened_data

def main(args=None):
    rclpy.init(args=args)
    node = EMGPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
