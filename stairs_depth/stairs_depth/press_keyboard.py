import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, tty, termios
import threading

class SpaceBarPublisher(Node):
    def __init__(self):
        super().__init__('space_bar_publisher')
        self.publisher_ = self.create_publisher(String, '/stairs_check', 10)

        # 사용자 입력을 비동기로 처리할 쓰레드 생성
        self.input_thread = threading.Thread(target=self.check_spacebar)
        self.input_thread.daemon = True
        self.input_thread.start()

    def check_spacebar(self):
        # 터미널 설정을 사용하여 스페이스 바 입력 감지
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                ch = sys.stdin.read(1)
                if ch == ' ':  # 스페이스 바 입력 감지
                    self.publish_aaa_message()
                elif ch == 'q':  # q 입력 감지
                    print("Exiting on 'q' key press.")
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def publish_aaa_message(self):
        msg = String()
        msg.data = "aaa"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "aaa"')

def main(args=None):
    rclpy.init(args=args)
    space_bar_publisher = SpaceBarPublisher()

    try:
        rclpy.spin(space_bar_publisher)
    except KeyboardInterrupt:
        print("Exiting on Ctrl+C.")
        rclpy.shutdown()
    finally:
        space_bar_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
