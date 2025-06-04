import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import numpy as np
import onnxruntime as ort
import json
#ros2 topic pub /target_label std_msgs/Int32 "{data: 0}"
class EMGClassifierNode(Node):
    def __init__(self):
        super().__init__('emg_classifier')

        config_path = '/workspace/EMG_drive_control/src/EMG_control/config.json'
        with open(config_path, 'r') as f:
            config = json.load(f)

        self.window_size = config.get('window_size', 1024)
        self.token_len = config.get('token_len', 1)
        self.num_channels = config.get('num_channels', 24)
        self.model_path = config.get('model_path', '/workspace/EMG_drive_control/8_channel.onnx')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/EMG_signal',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(Int32, '/predicted_gesture', 10)
        
        self.buffer = []

        self.session = ort.InferenceSession(self.model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

    def listener_callback(self, msg):
        emg_array = np.array(msg.data, dtype=np.float32)[:self.num_channels].reshape(1, self.num_channels)
        self.buffer.append(emg_array)
        if len(self.buffer) >= self.window_size:
            window_data = np.array(self.buffer[:self.window_size], dtype=np.float32)
            num_tokens=int(self.window_size/self.token_len)
            window_data=window_data.reshape(1, num_tokens, self.token_len, self.num_channels).astype(np.float32)
            #print(np.shape(window_data))
            self.make_prediction(window_data)
            self.buffer = []
            #print(self.buffer)
    def make_prediction(self, window_data):
        output = self.session.run([self.output_name], {self.input_name: window_data})[0]
        prediction = int(np.argmax(output, axis=1)[0])

        #self.get_logger().info(f'Predicted gesture ID: {prediction}')
        self.publisher.publish(Int32(data=prediction))  
            
def main(args=None):
    rclpy.init(args=args)
    node = EMGClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
