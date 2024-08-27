import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# # 깊이 그래프를 그리는 함수 추가  ------------------------------------durlqnxj
# def plot_depth_graph(diff_values, height=480, width=640):
#     return graph_img

class DiffArraySubscriber(Node):
    def __init__(self):
        super().__init__('diff_array_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/diff_array_msg',
            self.process_diff_array,
            10)
        self.subscription  # prevent unused variable warning

    def process_diff_array(self, msg):
        diff_array = np.array(msg.data)
        if len(diff_array) == 0:
            cv2.imshow('Depth Array', diff_array)
            cv2.waitKey(1)
            return
        height = 480; width = 640
        #original screan frame0
        graph_img = np.zeros((height, width, 3), dtype=np.uint8)
        mid_value = 240

        # 중간값을 기준으로 정규화
        normalized_diff = np.interp(diff_array, (np.min(diff_array), np.max(diff_array)), (0, height))
        
        for i in range(1, len(normalized_diff)):
            cv2.line(graph_img,
                    (int(i-1 * width / len(normalized_diff)), height - int(normalized_diff[i-1])+ mid_value),
                    (int(i * width / len(normalized_diff)), height - int(normalized_diff[i])+ mid_value),
                    (255, 0, 0), 2)

        cv2.imshow('Diff Array', graph_img)
        cv2.waitKey(1)
        

def main():
    rclpy.init()
    listener = DiffArraySubscriber()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()