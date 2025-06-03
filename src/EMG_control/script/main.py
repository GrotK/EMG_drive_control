import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import numpy as np
import onnxruntime as ort

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
        self.window_size = 1024  # dopasuj do modelu

        # Wczytanie modelu ONNX
        self.session = ort.InferenceSession('/workspace/EMG_drive_control/8_channel.onnx', providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

    def listener_callback(self, msg):
        emg_array = np.array(msg.data, dtype=np.float32)[:8].reshape(1, 8)
        self.buffer.append(emg_array)
            #print(emg_array[:8])
        #print(self.session.get_inputs()[0].shape)
        if len(self.buffer) >= self.window_size:
            window_data = np.array(self.buffer[:self.window_size], dtype=np.float32)
            window_data=window_data.reshape(1, 512, 2, 8).astype(np.float32)
            print(np.shape(window_data))
            self.make_prediction(window_data)
            self.buffer = self.buffer[self.window_size:]
            
    def make_prediction(self, window_data):
        output = self.session.run([self.output_name], {self.input_name: window_data})[0]
        prediction = int(np.argmax(output, axis=1)[0])

        self.get_logger().info(f'Predicted gesture ID: {prediction}')
        self.publisher.publish(Int32(data=prediction))  
            
def main(args=None):
    rclpy.init(args=args)
    node = EMGClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
