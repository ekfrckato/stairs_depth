import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, tty, termios
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time
import logging
from rclpy.qos import QoSProfile,  ReliabilityPolicy, HistoryPolicy

class StairsCheckSubscriber(Node):
    def __init__(self):
        super().__init__('stairs_check_subscriber')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_stair = self.create_subscription(String, 
                                                  '/stairs_check', 
                                                  self.stair_rocomotion_control, 
                                                  qos_profile)
        self.sub_stair

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
        self.DXL_ID = [10, 11, 12]
        self.DEVICENAME = '/dev/ttyCM'
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.DXL_MOVING_STATUS_THRESHOLD = 20  # 수정된 부분, 더 큰 값으로 설정

        self.COMM_SUCCESS = 0

        self.linear = 0
        self.linear_up = 0

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.group_pos_SyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)
        self.group_pos_SyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        self.group_vel_SyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_VEL, self.LEN_GOAL_VEL)
        self.group_vel_SyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_VEL, self.LEN_PRESENT_VEL)

        self.dxl_goal_positions = [1350, 1860, 2048]  # 각 모터별 목표 위치

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
        self.move_init_pos(self.DXL_ID[0], self.dxl_goal_positions[0])
        self.move_init_pos(self.DXL_ID[1], self.dxl_goal_positions[1])
        self.move_init_pos(self.DXL_ID[2], self.dxl_goal_positions[2])
        self.set_operating_mode(self.OPERATING_MODE_VELOCITY)

    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def Torque_on(self, DXL_ID):
        for i in range(len(self.DXL_ID)):
            # 현재 위치를 읽어온 후 그 위치로 목표 위치를 설정함
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_PRESENT_POSITION)
            
            # 현재 위치를 목표 위치로 설정하여 고정
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(dxl_present_position)),
                DXL_HIBYTE(DXL_LOWORD(dxl_present_position)),
                DXL_LOBYTE(DXL_HIWORD(dxl_present_position)),
                DXL_HIBYTE(DXL_HIWORD(dxl_present_position))
            ]
            dxl_addparam_result = self.group_pos_SyncWrite.addParam(self.DXL_ID[i], param_goal_position)
            if not dxl_addparam_result:
                print(f"[ID:{self.DXL_ID[i]}] groupSyncWrite addparam failed")
                quit()

            # 목표 위치로 명령 전송
            dxl_comm_result = self.group_pos_SyncWrite.txPacket()
            if dxl_comm_result != self.COMM_SUCCESS:
                print(f"[ID:{self.DXL_ID[i]}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")

            # 파라미터 클리어
            self.group_pos_SyncWrite.clearParam()

            # 토크 활성화
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != self.COMM_SUCCESS:
                print(f"[ID:{self.DXL_ID[i]}] TxRxResult Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"[ID:{self.DXL_ID[i]}] RxPacketError: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"Dynamixel#{self.DXL_ID[i]} torque enabled")

    def Torque_off(self, DXL_ID):
        for i in range(len(self.DXL_ID)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            if dxl_comm_result != self.COMM_SUCCESS:
                print(f"[ID:{self.DXL_ID[i]}] TxRxResult Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"[ID:{self.DXL_ID[i]}] RxPacketError: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"Dynamixel#{self.DXL_ID[i]} torque disabled")

    def move_init_pos(self, DXL_ID, dxl_goal_position):
        for i in range(len(self.DXL_ID)):
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID[i], self.ADDR_PRESENT_POSITION)
            print(f"[ID:{DXL_ID[i]}] Present Position: {dxl_present_position}, Goal Position: {dxl_goal_position}")

            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)),
                DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)),
                DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)),
                DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))
            ]
            
            dxl_addparam_result = self.group_pos_SyncWrite.addParam(DXL_ID[i], param_goal_position)
            if not dxl_addparam_result:
                print(f"[ID:{DXL_ID[i]}] groupSyncWrite addparam failed")
                quit()

            dxl_comm_result = self.group_pos_SyncWrite.txPacket()
            if dxl_comm_result != self.COMM_SUCCESS:
                print(f"[ID:{DXL_ID[i]}] {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            
            self.group_pos_SyncWrite.clearParam()
            # self.get_present_pos()

    def get_present_pos(self, DXL_ID):
        dxl_present_positions = [0] * len(self.DXL_ID)
        
        while True:
            for i in range(len(self.DXL_ID)):
                dxl_present_positions[i], dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                    self.portHandler, self.DXL_ID[i], self.ADDR_PRESENT_POSITION
                )
                print(f"[ID:{self.DXL_ID[i]}] Present Position: {dxl_present_positions[i]}")
            
            if all(abs(goal_pos - pos) < self.DXL_MOVING_STATUS_THRESHOLD for goal_pos, pos in zip(self.dxl_goal_positions, dxl_present_positions)):
                print("All motors reached the goal position.")
                break

            time.sleep(0.1)

    #torque off -> on
    def set_operating_mode(self, mode):
        time.sleep(0.5)
        self.Torque_off()
        for i in range(len(self.DXL_ID)):
            print(f"Setting operating mode to {mode} for Dynamixel ID {self.DXL_ID[i]}")
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, self.DXL_ID[i], self.ADDR_OPERATING_MODE, mode
            )

            if dxl_comm_result != self.COMM_SUCCESS:
                print(f"[ID:{self.DXL_ID[i]}] TxRxResult Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"[ID:{self.DXL_ID[i]}] RxPacketError: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"Dynamixel#{self.DXL_ID[i]}: Successfully set to mode {mode}")

            time.sleep(0.05)
            
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
            
            time.sleep(0.05)
            self.Torque_on()

    def set_vel(self, dxl_id, dxl_goal_velocity):
        if isinstance(dxl_id, int):
            dxl_id = [dxl_id]

        for i, dxl_id in enumerate(dxl_id):
            param_goal_velocity = [
                DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity[i])),
                DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity[i])),
                DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity[i])),
                DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity[i]))
            ]
            dxl_addparam_result = self.group_vel_SyncWrite.addParam(dxl_id, param_goal_velocity)
            if not dxl_addparam_result:
                print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
                quit()

        dxl_comm_result = self.group_vel_SyncWrite.txPacket()
        if dxl_comm_result != self.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        self.group_vel_SyncWrite.clearParam()
    
    #torque on -> off
    def stair_rocomotion_control(self, msg):
        self.Torque_on()
        Format = "%(asctime)s %(message)s"
        logging.basicConfig(filemode = "w", filename = "file.log", format = Format,  level=logging.DEBUG)

        logging.debug(msg.data)
        print(msg.data)
        
        turn_over_vel = [150]
        turn_back_vel = [-150]
        turn_linear_vel_down = [75, -75]
        turn_linear_vel_up = [-75, 75]
        turn_linear_vel_zero = [0, 0]

        stair_check_num = None
        if msg.data == 'over':      #E
            print("success receive msg")
            self.set_vel(self.DXL_ID[0], turn_over_vel)
            time.sleep(1)
            self.set_vel(self.DXL_ID[0], [0])

        # here, make a turn off - on signal for detecting stairs

        elif msg.data == 'back':    #Q
            print("success receive msg")
            self.set_vel(self.DXL_ID[0], turn_back_vel)
            time.sleep(1)
            
            # 위치 제어 모드로 변경
            self.set_operating_mode(self.OPERATING_MODE_POSITION)
            time.sleep(0.1)
            self.move_init_pos(self.DXL_ID[0], self.dxl_goal_positions[0])
            self.set_operating_mode(self.OPERATING_MODE_VELOCITY)

        elif msg.data == 'linear':      #I 
            if self.linear == 0:
                self.linear = 1
            else:
                self.linear = 0

            if self.linear % 2 == 1:
                self.set_vel(self.DXL_ID[1:3], turn_linear_vel_down)
            elif self.linear % 2 == 0:
                self.set_vel(self.DXL_ID[1:3], turn_linear_vel_zero)
                self.Torque_off(self.DXL_ID[1:3])

        elif msg.data == 'linear_up':   #O
            if self.linear_up == 0:
                self.linear_up = 1
            else:
                self.linear_up = 0

            if self.linear_up % 2 == 1:
                self.set_vel(self.DXL_ID[1:3], turn_linear_vel_up)
            elif self.linear_up % 2 == 0:
                self.set_vel(self.DXL_ID[1:3], turn_linear_vel_zero)
                self.Torque_off(self.DXL_ID[1:3])

        if msg.data == 'auto_overcome':      #E
            print("success receive msg")
            print("warning : !!! check the verlocity !!! ---- make vel <1.0 m/s>")
            
            try :
                self.set_vel(self.DXL_ID[0], turn_over_vel)
                time.sleep(1)
                self.Torque_off(self.DXL_ID[0])
                time.sleep(6)
                self.set_vel(self.DXL_ID[1:3], turn_linear_vel_down)
                time.sleep(4.5)
                self.set_vel(self.DXL_ID[1:3], turn_linear_vel_zero)
                self.Torque_off(self.DXL_ID[1:3])
                time.sleep(1.5)
                self.set_operating_mode(self.OPERATING_MODE_POSITION)
                self.move_init_pos(self.DXL_ID[1:3], self.dxl_goal_positions[2])
                self.set_operating_mode(self.OPERATING_MODE_VELOCITY)
                time.sleep(10)
            except KeyboardInterrupt:  # 키보드 인터럽트 발생 시
                print("KeyboardInterrupt detected, safely stopping the process.")
                self.Torque_off(self.DXL_ID[0:4])
                time.sleep(5)
        # here, make a turn off - on signal for detecting stairs

        print(f"linear : {self.linear}, linear_up : {self.linear_up}")

            


def main(args=None):
    rclpy.init(args=args)
    stairs_check = StairsCheckSubscriber()

    try:
        rclpy.spin(stairs_check)
    except KeyboardInterrupt:
        print('Depth array subscriber stopped cleanly')
        stairs_check.set_vel([0, 0])
        stairs_check.Torque_off(StairsCheckSubscriber.DXL_ID[1:3])
        print("KeyboardInterrupt detected, safely stopping the process.")
        StairsCheckSubscriber.Torque_off(StairsCheckSubscriber.DXL_ID[0:4])
        time.sleep(5)
    except BaseException as e:
        print(f'Exception in depth array subscriber: {e}')
        raise
    finally:
        stairs_check.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
