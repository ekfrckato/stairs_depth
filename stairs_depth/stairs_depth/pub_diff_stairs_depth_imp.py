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
        
        height = 240
        width = 320
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        if len(depth_array) == 0:
            cv2.imshow('Depth Array', img)
            cv2.waitKey(1)
            return
        
        x = np.arange(len(depth_array))
        y = np.array(depth_array)

        # 0값에 가까운 잡음 처리
        noise_value_num = [i for i in range(len(y)) if np.isclose(y[i], 0.0, atol=1e-6)] 
        for idx in noise_value_num:
            if idx == 0:
                y[idx] = y[idx + 1] if (idx + 1) < len(y) else y[idx]
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
        gradients = np.diff(y_scaled)
        print(gradients)
        print(y_scaled)
        # 퍼블리시 관련 부분은 그대로 유지
        gradients2list = gradients.tolist()
        gradient_msg = Float32MultiArray()
        gradient_msg.data = gradients2list
        self.publisher_diff_array.publish(gradient_msg)

        # 계단 여부 체크 후 퍼블리시
        index_wall_height = 0
        index_floor_N = 0
        stair_being_there = False
        negative_duration = 0  # 음수가 유지되는 기간
        positive_duration = 0  # 양수가 유지되는 기간
        min_negative_duration = 40  # 음수가 최소한 유지되어야 하는 기간
        min_positive_duration = 60  # 양수가 최소한 유지되어야 하는 기간
        pattern_count = 0
        required_pattern_count = 2  # 연속적인 패턴이 나타나는 최소 횟수
        state = 'neutral'  # 현재 상태를 추적 (neutral, negative, positive)

        for i in range(len(gradients)):
            if gradients[i] <= 0:
                negative_duration += 1
                if (state == 'positive' or state == 'neutral') and (positive_duration >= min_positive_duration):
                    if not (y_scaled[i] >= 130 and y_scaled[i] <= 240): 
                        # 양수에서 음수로 전환될 때, 양수가 일정 시간 이상 유지된 경우 패턴으로 인정
                        pattern_count += 1
                        positive_duration = 0
                state = 'negative'
            else:
                positive_duration += 1
                if (state == 'negative' or state == 'neutral') and (negative_duration >= min_negative_duration):
                    if not (y_scaled[i] >= 130 and y_scaled[i] <= 240):  # 130 ~ 145 값 무시
                        print( ": ", y_scaled[i]," : ", i)
                        # 음수에서 양수로 전환될 때, 음수가 일정 시간 이상 유지된 경우 패턴으로 인정
                        pattern_count += 1
                        negative_duration = 0
                    else:
                        continue
                    if pattern_count ==2:
                        detect_depth = y_scaled[i]
                        print(height - detect_depth) 
                state = 'positive'

            # 연속적인 패턴이 설정된 횟수 이상 발생하면 계단이 있다고 판단
            if pattern_count >= required_pattern_count:
                stair_being_there = True
                pattern_count = 0
                break

        if stair_being_there:
            print('계단이 있음')
            print(y_scaled[i]," : num : ", i)
            if 65 >= y_scaled[i] >= 60:
                stairs_check = String()
                stairs_check.data = 'over'
                self.publisher_stairs_check.publish(stairs_check)


        # xnew와 y_scaled 그래프 그리기 부분
        if np.size(y) == 0:
            xnew = np.linspace(0, 1, 1)
        else:
            xnew = np.linspace(0, np.size(y) - 1, num=np.size(y))

        prev_point = None
        max_xnew = np.max(xnew)
        if np.isnan(max_xnew) or max_xnew == 0:
            print("Invalid max value for xnew")
            return

        # 퍼블리시 제거, 그래프만 그리도록 변경
        for i, (x_val, y_val) in enumerate(zip(xnew, y_scaled)):
            x_curr = int(width - ((x_val / max_xnew) * width))
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

