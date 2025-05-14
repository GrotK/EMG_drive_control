import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import torch
import numpy as np

class EMGClassifierNode(Node):
    def __init__(self):
        super().__init__('emg_classifier')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/EMG_signal',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(Int32, '/predicted_gesture', 10)
        
        self.buffer = []
        self.window_size = 1024
        
        self.model = torch.load('/workspace/EMG_drive_control/emg_model.pt', map_location=torch.device('cpu'))
        self.model.eval()

    def listener_callback(self, msg):
        emg_array = np.array(msg.data, dtype=np.float32)

        self.buffer.extend(emg_array)

        if len(self.buffer) >= self.window_size:
            window_data = np.array(self.buffer[:self.window_size], dtype=np.float32)
            self.make_prediction(window_data)
            self.buffer = self.buffer[self.window_size:]
            
    def make_prediction(self, window_data):
        emg_tensor = torch.tensor(window_data).view(1, window_data.shape[0], -1)

        with torch.no_grad():
            output = self.model(emg_tensor)
            prediction = int(torch.argmax(output, dim=1).item())

        self.get_logger().info(f'Predicted gesture ID: {prediction}')

        self.publisher.publish(Int32(data=prediction))  
            
def main(args=None):
    rclpy.init(args=args)
    node = EMGClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
