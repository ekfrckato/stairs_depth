import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.interpolate import interp1d
import cv2
from scipy.signal import lfilter

class DepthArraySubscriber(Node):
    def __init__(self):
        super().__init__('depth_imp_img_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/depth_imp_img',
            self.opencv_graph,
            10)
        self.subscription  # prevent unused variable warning

    def opencv_graph(self, msg):
        xnew = msg.data[0]
        y_scaled = msg.data[1]

        height = 480
        width = 640
        img = np.zeros((height, width, 3), dtype=np.uint8)

        prev_point = None
        for i, (x_val, y_val) in enumerate(zip(xnew, y_scaled)):
            x_curr = int(width - ((x_val / np.max(xnew)) * width))
            y_curr = int(y_val)
            curr_point = (x_curr, y_curr)
            
            if prev_point is not None:
                cv2.line(img, prev_point, curr_point, (0, 255, 0), 2)
            
            prev_point = curr_point
        
        cv2.imshow('Depth Array', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    depth_array_subscriber = DepthArraySubscriber()

    try:
        rclpy.spin(depth_array_subscriber)
    except KeyboardInterrupt:
        print('img ^_^ ')
    except BaseException as e:
        print(f'Exception in depth array subscriber: {e}')
        raise
    finally:
        depth_array_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
