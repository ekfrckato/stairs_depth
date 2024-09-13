import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import numpy as np
from scipy.interpolate import interp1d
import cv2
from scipy.signal import lfilter

class DepthArraySubscriber(Node):
    def __init__(self):
        super().__init__('depth_array_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray,'/depth_array_msg',self.opencv_graph,10)
        self.subscription  # prevent unused variable warning
        #publisher setting
        self.publisher_diff_array = self.create_publisher(Float32MultiArray, '/diff_array_msg', 10)
        self.publisher_depth_img = self.create_publisher(Float32MultiArray, '/depth_imp_img', 10)
        self.publisher_stairs_check = self.create_publisher(String , '/stairs_check', 10)
        self.get_data = []
        
    def capture_data(self, gradients):
        self.get_data.add(gradients) 
        
    
    def opencv_graph(self, msg):
        depth_array = msg.data
        
        height = 480
        width = 640
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        if len(depth_array) == 0:
            cv2.imshow('Depth Array', img)
            cv2.waitKey(1)
            return
        
        x = np.arange(len(depth_array))
        y = np.array(depth_array)

        noise_value_num = [i for i in range(len(y)) if np.isclose(y[i], 0.0, atol=1e-6)] 
        for idx in noise_value_num:
            if idx == 0:
                y[idx] = y[idx + 1] if idx + 1 < len(y) else y[idx]
            elif idx == len(y) - 1:
                y[idx] = y[idx - 1]
            else:
                y[idx] = y[idx + 1] if not np.isclose(y[idx + 1], 0.0, atol=1e-6) else y[idx - 1]

        # 스케일 조정을 위해 y 값 조정
        diff = np.max(y) - np.min(y)
        scale_factor = height / diff if diff > 0 else 1
        y_scaled = (y - np.min(y)) * scale_factor
        y_scaled = height - y_scaled  # Y축 반전 적용

        # 기울기 계산
        #len(gradients) = 360
        #np.diff() : 
        gradients = np.diff(y_scaled)  # y_scaled에 대한 기울기 계산
        print(gradients)

        #publish
        gradients2list=gradients.tolist()
        gradient_msg = Float32MultiArray()
        gradient_msg.data = gradients2list
        self.publisher_diff_array.publish(gradient_msg)

        #hing hagisiro
        i = 0
        index_wall_N = 0
        index_floor_N = 0
        wall_diff = 10
        wall_height = 10
        floor_diff = 22
        floor_width = 30
        stair_being_there = False
        for i in range(len(gradients)):
            if i is None : gradients[i:] = 0
            for j, x in enumerate(gradients[i:]):
                # j = correct the num 
                if x > wall_diff:
                    index_wall_N =+ 1
                    break

            if j > wall_height:
                for k, y in enumerate(gradients[j+i:]):
                    if (y > floor_diff) & (k < floor_width):
                        index_floor_N =+ 1
            if (index_wall_N >2) & (index_floor_N > 1):
                stair_being_there = True

        if stair_being_there is True :
            print('there is stair')
            stairs_check = String()
            stairs_check.data = 'over'
            self.publisher_stairs_check.publish(stairs_check)

        #publish
        if np.size(y) == 0 :
            xnew = np.linspace(0, 1, 1)
        else :
            xnew = np.linspace(0, np.size(y) - 1, num=np.size(y))
        xnew.tolist()
        depth_imp_img = Float32MultiArray([xnew, y_scaled])
        depth_imp_img.data = depth_imp_img
        self.publisher_depth_img.publish(depth_imp_img)
        prev_point = None
        # xnew와 y를 사용해 그래프를 그릴 수 있도록 데이터 스케일 조정
        
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
        print('Depth array subscriber stopped cleanly')
    except BaseException as e:
        print(f'Exception in depth array subscriber: {e}')
        raise
    finally:
        depth_array_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
