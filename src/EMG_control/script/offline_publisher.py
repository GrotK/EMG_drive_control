import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import numpy as np
import random

class EMGPublisherNode(Node):
    def __init__(self):
        super().__init__('emg_publisher')

        self.publisher = self.create_publisher(Float32MultiArray, '/EMG_signal', 10)
        self.subscription = self.create_subscription(Int32, '/target_label', self.label_callback, 10)

        self.data = np.load('/workspace/sequential_X/sequential_X.npz')['arr']  # (N, 1024, 24)
        self.labels = np.argmax(np.load('/workspace/sequential_X/sequential_y.npz')['arr'], axis=1)  # (N,)
        
        self.target_label = 0
        self.state = 'idle'
        self.sample_size = 10
        self.sample_sequence = None
        self.sample_index = 0

        self.timer = self.create_timer(0.001, self.publish_sample)

    def label_callback(self, msg):
        label = msg.data
        if self.state=='idle':
            self.target_label = label
            self.get_logger().info(f'Received target label: {label}')
            self.select_sequence_for_label(label)
        if label != self.target_label:
            self.get_logger().info(f'Received target label: {label}')
            self.sequence=None
            self.select_sequence_for_label(label)
            self.target_label=label

    def find_label_sequences(self, label):
        sequences = []
        start = None
        for i, lab in enumerate(self.labels):
            if lab == label:
                if start is None:
                    start = i
            else:
                if start is not None:
                    sequences.append((start, i))
                    start = None
        if start is not None:
            sequences.append((start, len(self.labels)))
        return sequences

    def select_sequence_for_label(self, label):
        sequences = self.find_label_sequences(label)
        if not sequences:
            self.get_logger().warn(f"No sequences found for label {label}")
            return

        selected_start, selected_end = random.choice(sequences)
        selected_seq = self.data[selected_start:selected_end]  
        self.sample_sequence = selected_seq.reshape(-1, 24)    
        self.sample_index = 0
        self.state = 'sending'
        #self.get_logger().info(f"Selected label {label} sequence: [{selected_start}, {selected_end})")

    def publish_sample(self):
        if self.state != 'sending' or self.sample_sequence is None:
            return

        if self.sample_index < len(self.sample_sequence):
            msg = Float32MultiArray()
            msg.data = self.sample_sequence[self.sample_index].flatten().tolist()  
            self.publisher.publish(msg)
            #self.get_logger().info(f"published {self.sample_index } out of: {len(self.sample_sequence)} samples for {self.target_label} label")
            self.sample_index += 1
        else:
            self.get_logger().info('Finished sending sequence.')
            self.sample_sequence = None
            self.sample_index = 0
            self.state = 'idle'
            self.target_label = 0
            self.select_sequence_for_label(self.target_label)

        
        
        
def main(args=None):
    rclpy.init(args=args)
    node = EMGPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
