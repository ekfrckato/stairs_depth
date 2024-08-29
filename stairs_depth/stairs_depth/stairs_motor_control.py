import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, tty, termios
import threading
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time
import ctypes
import logging
from rclpy.qos import QoSProfile, qos_profile_sensor_data,  ReliabilityPolicy, HistoryPolicy

class StairsCheckSubscriber(Node):
    def __init__(self):
        super().__init__('stairs_check_subscriber')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_stair = self.create_subscription(String, 
                                                  '/stairs_check', 
                                                  self.stair_rocomotion_control, 
                                                  qos_profile)
        self.sub_stair

        # 타이머 콜백에서 인자를 전달하지 않는 별도의 함수 호출
        # self.timer = self.create_timer(1.0, self.stair_rocomotion_control)

        self.last_signal_time = time.time()  # 마지막으로 신호를 받은 시간을 초기화

        self.MY_DXL = 'X_SERIES'
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.LEN_GOAL_POSITION = 4
        self.ADDR_PRESENT_POSITION = 132
        self.LEN_PRESENT_POSITION = 4
        self.ADDR_GOAL_VEL = 104
        self.LEN_GOAL_VEL = 4
        self.ADDR_PRESENT_VEL = 128
        self.LEN_PRESENT_VEL = 4
        self.ADDR_OPERATING_MODE = 11
        self.OPERATING_MODE_VELOCITY = 1
        self.OPERATING_MODE_POSITION = 3
        self.BAUDRATE = 57600
        self.PROTOCOL_VERSION = 2.0
        self.DXL_ID = [3]
        self.DEVICENAME = '/dev/ttyACM0'
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.DXL_MOVING_STATUS_THRESHOLD = 10

        self.COMM_SUCCESS = 0

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.group_pos_SyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)
        self.group_pos_SyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        self.group_vel_SyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_VEL, self.LEN_GOAL_VEL)
        self.group_vel_SyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_VEL, self.LEN_PRESENT_VEL)

        self.dxl_goal_position = 2048

        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            self.getch()
            quit()

        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            self.getch()
            quit()

        self.set_operating_mode(self.OPERATING_MODE_POSITION)
        self.Torque_on()
        self.move_init_pos()
        self.set_operating_mode(self.OPERATING_MODE_VELOCITY)
        self.Torque_on()

    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def Torque_on(self):
        # 토크를 활성화합니다.
        for i in range(len(self.DXL_ID)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != self.COMM_SUCCESS:
                print(f"[ID:{self.DXL_ID[i]}] TxRxResult Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"[ID:{self.DXL_ID[i]}] RxPacketError: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"Dynamixel#{self.DXL_ID[i]} torque enabled")

    def Torque_off(self):
        # 토크를 활성화합니다.
        for i in range(len(self.DXL_ID)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            if dxl_comm_result != self.COMM_SUCCESS:
                print(f"[ID:{self.DXL_ID[i]}] TxRxResult Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"[ID:{self.DXL_ID[i]}] RxPacketError: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"Dynamixel#{self.DXL_ID[i]} torque enabled")

    def move_init_pos(self):

        # 목표 위치(2048)를 각 모터에 설정하는 파라미터를 준비합니다.
        param_goal_position = [
            DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position)),
            DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position)),
            DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position)),
            DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position))
        ]
        
        # 각 모터에 대해 목표 위치를 설정하고, 그 결과를 확인합니다.
        for i in range(len(self.DXL_ID)):
            print(f"Setting goal position for Dynamixel ID {self.DXL_ID[i]}")
            dxl_addparam_result = self.group_pos_SyncWrite.addParam(self.DXL_ID[i], param_goal_position)
            if not dxl_addparam_result:
                print(f"[ID:{self.DXL_ID[i]}] groupSyncWrite addparam failed")
                quit()

        # 설정된 목표 위치를 모든 모터에 동기화하여 적용합니다.
        dxl_comm_result = self.group_pos_SyncWrite.txPacket()
        if dxl_comm_result != self.COMM_SUCCESS:
            print(f"[ID:{self.DXL_ID[i]}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")

        # 파라미터를 초기화합니다.
        self.group_pos_SyncWrite.clearParam()
        self.get_present_pos()

    def get_present_pos(self):
        # 모터의 현재 위치를 모니터링합니다.
        for i in range(len(self.DXL_ID)):
            dxl_addparam_result = self.group_pos_SyncRead.addParam(self.DXL_ID[i])
            if not dxl_addparam_result:
                print(f"[ID:{self.DXL_ID[i]}] groupSyncRead addparam failed")
                quit()

        # 현재 위치를 계속 읽어오는 루프입니다.
        while True:
            dxl_comm_result = self.group_pos_SyncRead.txRxPacket()
            if dxl_comm_result != self.COMM_SUCCESS:
                print(f"[ID:{self.DXL_ID[i]}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            else:
                for i in range(len(self.DXL_ID)):
                    dxl_present_position = self.group_pos_SyncRead.getData(self.DXL_ID[i], self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
                    print(f"[ID:{self.DXL_ID[i]}] Present Position: {dxl_present_position}")

            # 모터가 목표 위치에 도달했는지 확인하는 조건문입니다.
            if all(abs(self.dxl_goal_position - self.group_pos_SyncRead.getData(id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)) < self.DXL_MOVING_STATUS_THRESHOLD for id in self.DXL_ID):
                print("All motors reached the goal position.")
                break

            time.sleep(0.1)

    def set_operating_mode(self, mode):
        self.Torque_off()
        for i in range(len(self.DXL_ID)):
            print(f"Setting operating mode to {mode} for Dynamixel ID {self.DXL_ID[i]}")

            # 모드를 설정하기 위한 시도
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, self.DXL_ID[i], self.ADDR_OPERATING_MODE, mode
            )

            # 커뮤니케이션 결과 및 에러 체크
            if dxl_comm_result != self.COMM_SUCCESS:
                print(f"[ID:{self.DXL_ID[i]}] TxRxResult Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"[ID:{self.DXL_ID[i]}] RxPacketError: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"Dynamixel#{self.DXL_ID[i]}: Successfully set to mode {mode}")

            time.sleep(0.05)  # 모드 변경 후 충분한 지연 시간을 추가
            
            # 현재 모터의 운영 모드를 다시 읽어 확인
            current_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
                self.portHandler, self.DXL_ID[i], self.ADDR_OPERATING_MODE
            )
            if dxl_comm_result == self.COMM_SUCCESS and dxl_error == 0:
                if current_mode == mode:
                    print(f"Dynamixel#{self.DXL_ID[i]} is confirmed to be in mode {current_mode}")
                else:
                    print(f"[ID:{self.DXL_ID[i]}] Operating mode mismatch. Expected: {mode}, Got: {current_mode}")
            else:
                print(f"[ID:{self.DXL_ID[i]}] Failed to read operating mode. {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            
            time.sleep(0.05)  # 모드 확인 후 추가 지연


    # def timer_callback(self):
    #     # 타이머에서 호출할 때는 빈 메시지로 stair_rocomotion_control 호출
    #     empty_msg = String()
    #     empty_msg.data = "aaa"  # 예시로 'aaa'를 메시지로 사용
    #     self.stair_rocomotion_control(empty_msg)

    def set_vel(self,dxl_goal_velocity):
        for i in range(len(self.DXL_ID)):
            param_goal_velocity = [
                DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity[i])),
                DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity[i])),
                DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity[i])),
                DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity[i]))
            ]
            dxl_addparam_result = self.group_vel_SyncWrite.addParam(self.DXL_ID[i], param_goal_velocity)
            if not dxl_addparam_result:
                print("[ID:%03d] groupSyncWrite addparam failed" % self.DXL_ID[i])
                quit()

        dxl_comm_result = self.group_vel_SyncWrite.txPacket()
        if dxl_comm_result != self.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        self.group_vel_SyncWrite.clearParam()
    
    def stair_rocomotion_control(self, msg):
        self.Torque_on()
        Format = "%(asctime)s %(message)s"
        logging.basicConfig(filemode = "w", filename = "file.log", format = Format,  level=logging.DEBUG)

        logging.debug(msg.data)
        print(msg.data)

        if msg.data == 'over':
            stair_check_num = 1
        elif msg.data == 'back':
            stair_check_num = 3
        else:
            stair_check_num = 0

        turn_over_vel = [150]  # Set desired velocities for both motors
        turn_back_vel = [-150]
        if stair_check_num == 1:
            print("success receive msg")
            self.set_vel(turn_over_vel)  # 2초 동안 모터를 움직임
            time.sleep(2)  # 2초 동안 대기

            self.set_vel([0, 0])  # 속도 0으로 설정하여 멈춤
            self.Torque_off()  # 4초 동안 토크를 끈 상태로 유지
        elif stair_check_num == 3:
            print("success receive msg")
            self.set_vel(turn_back_vel)  # 2초 동안 모터를 움직임
            time.sleep(2)  # 2초 동안 대기

            self.set_vel([0, 0])  # 속도 0으로 설정하여 멈춤
            self.Torque_off()  # 4초 동안 토크를 끈 상태로 유지

def main(args=None):
    rclpy.init(args=args)

    stairs_check = StairsCheckSubscriber()

    try:
        rclpy.spin(stairs_check)
    except KeyboardInterrupt:
        print('Depth array subscriber stopped cleanly')
        stairs_check.set_vel([0, 0])  # 속도 0으로 설정
        stairs_check.Torque_off()  # 토크 오프
    except BaseException as e:
        print(f'Exception in depth array subscriber: {e}')
        raise
    finally:
        stairs_check.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
