import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray

from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import cv2
import pyrealsense2 as rs2

if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

# 깊이 그래프를 그리는 함수 추가  
def plot_depth_graph(depth_values, height=480, width=640):
    graph_img = np.zeros((height, width, 3), dtype=np.uint8)
    normalized_depth = np.interp(depth_values, (0, np.max(depth_values)), (0, height))

    for i in range(1, len(normalized_depth)):
        cv2.line(graph_img,
                 (int((i-1) * width / len(normalized_depth)), height - int(normalized_depth[-i+1])),
                 (int(i * width / len(normalized_depth)), height - int(normalized_depth[-i])),
                 (255, 0, 0), 2)
    return graph_img

class ImageListener(Node):
    def __init__(self, depth_image_topic, depth_info_topic):
        stairs_depth = os.path.basename(sys.argv[0]).split('.')[0]
        super().__init__(stairs_depth)
        self.bridge = CvBridge()
        
        self.sub = self.create_subscription(msg_Image, depth_image_topic, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        self.intrinsics = None
        #publisher setting
        self.publisher_ = self.create_publisher(Float32MultiArray, '/depth_array_msg', 10)

    def imageDepthCallback(self, data):
        try:
            # 원본 깊이 이미지를 OpenCV 형식으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            # 깊이 이미지를 0에서 255 사이로 스케일링
            depth_image_scaled = cv2.convertScaleAbs(cv_image, alpha=0.03)
            # 깊이 이미지에 컬러맵 적용
            depth_colormap = cv2.applyColorMap(depth_image_scaled, cv2.COLORMAP_JET)

            # 중앙 열의 깊이 값 추출 (이전 단계와 동일)
            height, width = cv_image.shape
            center_column = width // 2
            central_column_depth_values = cv_image[:, center_column]
            # 깊이 그래프 이미지 생성
            depth_graph_img = plot_depth_graph(central_column_depth_values, height=480, width=640)
    
            depth_array_msg = Float32MultiArray()
            # depth_array_msg.data = central_column_depth_values.tolist() 
            depth_array_msg.data = [float(value) for value in np.flip(central_column_depth_values)]
            depth_array_msg.reverse()
            self.publisher_.publish(depth_array_msg)

            print("test")

            # 깊이 그래프 및 컬러맵 적용된 깊이 이미지 표시
            cv2.imshow('Depth Graph', depth_graph_img)
            cv2.imshow('Depth Image', depth_colormap)  # 수정된 깊이 이미지를 표시
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            self.intrinsics.model = rs2.distortion.brown_conrady
            self.intrinsics.coeffs = list(cameraInfo.d)
        except CvBridgeError as e:
            print(e)
            return

def main():
    rclpy.init()
    depth_image_topic = '/camera/camera/depth/image_rect_raw'
    depth_info_topic = '/camera/camera/depth/camera_info'

    print ()
    print ('show_center_depth.py')
    print ('--------------------')
    print ('App to demontrate the usage of the /camera/depth topics.')
    print ()
    print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    print ('Application then calculates and print the range to the closest object.')
    print ('If intrinsics data is available, it also prints the 3D location of the object')
    print ()
    
    listener = ImageListener(depth_image_topic, depth_info_topic)
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
