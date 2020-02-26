#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os

import time
import numpy as np
import math

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),'robot')))
from Parent_Robot import Parent_Robot
from robot_slave import Robot_Slave

from RPP_functions_1224 import get_polar_2_3D_nor
import glob

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),'gripper')))
from keti_schunk_gripper import KetiSchunk
from keti_zimmer_gripper import KetiZimmer

#로봇 인스턴스 객체는 구현될 로봇들의 함수원형 및 기본적인 맴버 변수 원형만 가지고 있습니다.
#각 로봇의 마스터는 해당 클래스를 상속받아 구현하면 됨.

class robot_instance:
    #로봇의 마스터 코드를 구현시 아래의 함수를 복사하여 구현하면 됨
    move_joint_robot = []   #로봇을 조인트이동하는 명령
    move_pos_robot = []     #로봇을 입력받은 포즈로 이동하게 하는 명령
    move_pos_queue = []     #로봇이 포즈 리스트를 받으면 순차적으로 이동하게 하는 명령어
    move_joint_queue = []   #로봇을 조인트 리스트를 받으면 순차적으로 이동하게 하는 명령어
    move_pos_circle = []    #로봇이 해당 포즈까지 원형으로 이동하게 하는 명령어
    move_jog_robot = []     #로봇이 조그이동을 하게 하는 명령어

    robot_stop = []         #로봇을 정지시키는 명령.

    #아래 코드는 선택적으로 구현
    grip = []               #로봇이 그리퍼를 컨트롤 하는 코드.

    #아래 코드는 구현할 필요 없음. (공통부분)
    #get_robot_pos = []      #로봇의 현재 값을 받아옴
    #get_grip_success = []   #로봇이 그리퍼의 성공여부를 받아오는 코드,

    #인스턴스 클래스의 맴버 변수.
    #복사할 필요 없음.
    m_Robot_Slave = Parent_Robot()


class robot_master:
    #맴버 변수_ 설정

    #맴버함수_변수
    robot_module = [robot_instance()]   #추후에 다수의 로봇이 사용할 경우를 감안하여 리스트로 관리함.

    #로봇의 생성자.
    def __init__(self, robot_name = [], gripper_name = []):
        if(robot_name != []):
            self.init_robot(robot_name = robot_name)
            time.sleep(1)
        if(gripper_name != []):
            self.init_gripper(gripper_name = gripper_name)
            # print('그리퍼가 따로 설정되지 않으면, ros기본 그리퍼와 연결 시킨다.')

    def init_robot(self, robot_name = ''):
        if(robot_name == ''):
            print('로봇 이름이 명시되지 않았습니다.')
            return
        else:
            self.robot_module[0] = Robot_Slave(robot_name)
        print('적절한 로봇 모듈로 초기화 시킨다.')

    def init_gripper(self, gripper_name):
        print('적절한 그리퍼 모듈로 초기화 시킨다.')
#        print('하지만 구현은 안했다.')
        if gripper_name == 'schunk_gripper':
            self.robot_module[0].grip = KetiSchunk()
            self.robot_module[0].grip.init()
        if gripper_name == 'zimmer_gripper':
            self.robot_module[0].grip = KetiZimmer()
            self.robot_module[0].grip.init()

    #명령어 코드
    #명령어에 사용되는 좌표는 두산 로보틱스M기준의 좌표로 전달된다.
    #해당 명령어는 메인마스터에서 호출된다.
    def move_joint_robot(self,posj, vel=100, acc = 100, time = 0, moveType = 'Joint', syncType = 0, index_robot = 0):
        self.robot_module[index_robot].move_joint_robot(posj, vel = vel, acc = acc, time = time, moveType = moveType, syncType = syncType)

    def move_pos_robot(self, posx, time = 0.0, vel=[10,10], acc= [50,50], moveType = 'Line', syncType = 0, index_robot = 0):
        self.robot_module[index_robot].move_pos_robot(posx, vel = vel, acc = acc, time = time, moveType = moveType, syncType = syncType)

    def move_pos_queue(self, pos_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0, index_robot = 0):
        self.robot_module[index_robot].move_pos_queue(pos_list, vel=vel, acc=acc, time=time, mod=mod, ref=ref, vel_opt=vel_opt, syncType=syncType)

    def move_joint_queue(self, joint_list, vel=400, acc=400, time=5, mod=0, ref=0, vel_opt=0, syncType=0, index_robot = 0):
        self.robot_module[index_robot].move_joint_queue(joint_list, vel=vel, acc=acc, time=time, mod=mod, ref=ref, vel_opt=vel_opt, syncType=syncType)

    def move_pos_circle(self, waypos, destpos, vel=[50, 50], acc=[30, 30], syncType=0, index_robot = 0):
        self.robot_module[index_robot].move_pos_circle(waypos, destpos, vel=vel, acc=acc, syncType=syncType)

    def move_jog_robot(self, JOG_AXIS, REF, SPEED, index_robot = 0):
        self.robot_module[index_robot].move_jog_robot(JOG_AXIS, REF, SPEED)

    #
    def get_cur_pos(self, index_robot = 0):
        return self.robot_module[index_robot].get_cur_pos()

    def grip(self, mode = 'grip', pos = 0):
        #그립 코드는 언제나 최대파워, 최대 스피드로 집게 구현할 것임.
        print('별도의 그리퍼 함수가 선언되지 않으면, {}가 호출됩니다.')

    def grip_release(self):
        print('별도의 그리퍼 함수가 선언되지 않으면, {}가 호출됩니다.')

    def get_grip_success(self, index_robot = 0):
        print('별도의 그리퍼 함수가 선언되지 않으면, {}가 호출됩니다.')
        return False


    #공통함수.
    def get_line_based_tcp(self, X_term, Y_term, Z_term, base_pos=[0, 0, 0, 0, 0, 0], index_robot = 0):
        if (base_pos[0] == 0):
            ABC = self.robot_module[index_robot].get_cur_pos()  # self.state_robot_pos
        else:
            ABC = base_pos
        # print(ABC)
        nor = get_polar_2_3D_nor(ABC[3], ABC[4], ABC[5])
        # print('NO',nor)
        for i in range(3):
            ABC[i] = ABC[i] + nor[0][i] * X_term + nor[1][i] * Y_term + nor[2][i] * Z_term
        return ABC

    # TCP기준으로 움직이는 걸 전제로 한다.
    def move_line_based_tcp(self, X_term, Y_term, Z_term, base_pos=[0, 0, 0, 0, 0, 0], index_robot = 0):
        self.robot_module[index_robot].move_pos_robot(self.get_line_based_tcp(X_term, Y_term, Z_term, base_pos=base_pos,index_robot= index_robot), syncType = 1)

    def move_rotate_based_tcp(self, rotate, index_robot = 0):
        ABC = self.robot_module[index_robot].get_cur_pos()  # self.state_robot_pos
        ABC[5] += rotate
        self.robot_module[index_robot].move_pos_robot(ABC, syncType = 1)

    def move_line_based_tcp_and_rotate(self, X_term, Y_term, Z_term, rotate, base_pos=[0, 0, 0, 0, 0, 0], time=2.0,
                                       vel=[100, 100], acc=[100, 100], index_robot = 0):
        temp_ABC = self.get_line_based_tcp_and_rotate(X_term, Y_term, Z_term, rotate, base_pos=base_pos, index_robot= index_robot)
        self.robot_module[index_robot].move_pos_robot(temp_ABC, time=time,  syncType = 1)
        return temp_ABC

    def get_line_based_tcp_and_rotate(self, X_term, Y_term, Z_term, rotate, base_pos=[0, 0, 0, 0, 0, 0], index_robot = 0):
        if (base_pos[0] == 0):
            ABC = self.robot_module[index_robot].get_cur_pos()  # self.state_robot_pos
        else:
            ABC = base_pos
        # print('Cur_POs is... %s'%str(ABC))
        nor = RPP_functions.get_polar_2_3D_nor(ABC[3], ABC[4], ABC[5])
        # print('Curr_ Nor is..', nor)
        for i in range(3):
            ABC[i] = ABC[i] + nor[0][i] * X_term + nor[1][i] * Y_term + nor[2][i] * Z_term
        ABC[5] += rotate
        return ABC

    def move_rotate_x_based_tcp(self, xrotate, index_robot = 0):
        rotated_norvec, temp_pos = self.get_rotate_x_based_tcp(xrotate, index_robot= index_robot)
        State_Cur_Pos = self.robot_module[index_robot].get_cur_pos()
        State_Cur_Pos[3] = temp_pos[0]
        State_Cur_Pos[4] = temp_pos[1]
        State_Cur_Pos[5] = temp_pos[2]
        self.robot_module[index_robot].move_pos_robot(State_Cur_Pos, syncType = 1)

    # 보는 위치 기준 이동을 위해
    def get_rotate_x_based_tcp(self, x_rotate, base_pos=[0, 0, 0, 0, 0, 0], index_robot = 0):
        if (base_pos[0] == 0):
            ABC = self.robot_module[index_robot].get_cur_pos()  # self.state_robot_pos
        else:
            ABC = base_pos
        nor_vec1, nor_vec2, nor_vec3 = RPP_functions.get_polar_2_3D_nor(ABC[3], ABC[4], ABC[5])
        # nor_vec1 = RPP_functions.get_polar_2_nor([1,0,0], [ABC[3],ABC[4],ABC[5]])
        # nor_vec2 = RPP_functions.get_polar_2_nor([0, 0, 1], [ABC[3], ABC[4], ABC[5]])
        # print('current direction vec is...%s' % str(nor_vec1))
        # 방향벡터를 위기준으로 틀어주는 회전벡터를 구해준다.
        rota_matrix = RPP_functions.get_rotation_matrix(nor_vec1, x_rotate)

        # 회전 벡터를 현재 보는 방향(z축) 벡터를 틀어준다.
        rotated_norvec = RPP_functions.Rotate_Nor(nor_vec3, rota_matrix)
        rotated_norvec2 = rotated_norvec / np.sqrt(np.sum(rotated_norvec ** 2))
        # print('Rotated Norvec == %s (%s)' %(str(rotated_norvec2),str(rotated_norvec)))
        ret_ABC = RPP_functions.calculaion_nor_2_polar(rotated_norvec2)

        ret_ABC[2] = RPP_functions.get_C_using_inv_rotate(nor_vec1, ret_ABC[0], ret_ABC[1])
        return rotated_norvec2, ret_ABC

    def move_grip_move(self, X_term, Y_term, Z_term, base_pos=[0, 0, 0, 0, 0, 0], index_robot = 0):
        if (base_pos[0] == 0):
            ABC = self.robot_module[index_robot].get_cur_pos()  # self.state_robot_pos
        else:
            ABC = base_pos

        # self.grip()
        time.sleep(0.1)
        nor = RPP_functions.get_polar_2_3D_nor(ABC[3], ABC[4], ABC[5])
        for i in range(3):
            ABC[i] = ABC[i] + nor[0][i] * X_term + nor[1][i] * Y_term + nor[2][i] * Z_term
        self.robot_module[index_robot].move_pos_robot(ABC, syncType = 0)
        # time.sleep(0.2) #셕션 체크 후, 센서값 대기를 위한.
        start_time = time.time()  ##정지에 의한 조건에 사용된다.
        last_time = time.time()
        l_condition_time = 1.0  # 해당 초만큼 후 부터 충돌 체크.
        l_condition_speed = 0.015 * self.robot_module[index_robot].m_Robot_Slave.robot_config['time_rate']  # 이 속도보다 작다면 정지다.
        while (True):
            debug_bool = self.get_grip_success(index_robot=index_robot)
            # print(debug_bool)
            # 속도에 의한 정지 체크.
            if (last_time - start_time < l_condition_time):
                last_time = time.time()
            else:
                if (self.speed < l_condition_speed):
                    self.robot_module[index_robot].robot_stop()
                    self.robot_module[index_robot].grip_release()
                    print('\t[Tool Signal] [Warning] Suction stopped by Speed]')
                    return
            if (debug_bool):
                self.robot_module[index_robot].robot_stop()
                print('\t[Tool Signal] Suction stopped by Vacuum')
                return

    def message_comm(self, msg=-1):
        self.mm_msg = msg

if __name__ == '__main__':
    m_robot_master = robot_master(robot_name='a0509')
    # m_robot_master = robot_master(robot_name='a0509', gripper_name='zimmer_gripper')]

    move_joint = [180.0, 0.0, -90.0, 180.0, 90.0, 0.0]
    m_robot_master.move_joint_robot(move_joint, vel=20, acc=20, syncType=1, time=2)
    time.sleep(5)

    move_pose = [207.0, 0, 440.5, 35.434, -180, 35.434]
    m_robot_master.move_pos_robot(move_pose, vel=[100,100], acc=[1000,1000], syncType=1, time=2)
    time.sleep(5)

    # move_pose = [507.0, 0, 440.5, 35.434, -180, 35.434]
    # m_robot_master.move_pos_robot(move_pose, vel= velx, acc = accx, syncType=0, time=5)
    move_pose1 = [407.0, 0, 440.5, 35.434, -180, 35.434]
    move_pose2 = [507.0, 0, 440.5, 35.434, -180, 35.434]
    move_list = [move_pose1, move_pose2]
    m_robot_master.move_pos_queue(move_list, vel=100, acc=200, syncType=1, time=3)
    time.sleep(5)

    move_joint1 = [180.0, 0.0, -90.0, 180.0, 90.0, 0.0]
    move_joint2 = [180.0, 0.0, -120.0, 180.0, 60.0, 0.0]
    move_list = [move_joint1, move_joint2]
    m_robot_master.move_joint_queue(move_list, vel=100, acc=200, syncType=1, time=3)
    time.sleep(5)

    move_joint = [180.0, 0.0, -90.0, 180.0, 45.0, 0.0]
    m_robot_master.move_joint_robot(move_joint, vel=20, acc=20, syncType=1, time=2)
    time.sleep(5)

    m_robot_master.move_line_based_tcp(0, 0, 150)
    time.sleep(5)

    # move_pose1 = [454.681, 0.0, 476.819, 180.0, -135.0, 180.0]
    # sp = [0, 0, 150]
    # z1 = 180
    # y = -135
    # z2 = 180
    # Rz1 = np.array([[math.cos(math.radians(z1)), -math.sin(math.radians(z1)), 0],
    #                [math.sin(math.radians(z1)), math.cos(math.radians(z1)), 0],
    #                [0, 0, 1]])
    # Ry = np.array([[math.cos(math.radians(y)), 0, math.sin(math.radians(y))],
    #               [0, 1, 0],
    #               [-math.sin(math.radians(y)), 0, math.cos(math.radians(y))]])
    # Rz2 = np.array([[math.cos(math.radians(z2)), -math.sin(math.radians(z2)), 0],
    #                [math.sin(math.radians(z2)), math.cos(math.radians(z2)), 0],
    #                [0, 0, 1]])
    # R_z1_y = np.matmul(Rz1, Ry)
    # R = np.matmul(R_z1_y, Rz2)
    # s = np.matmul(R, sp)
    # # print s
    # move_pose2 = [move_pose1[0] + s[0], move_pose1[1] + s[1], move_pose1[2] + s[2], 180.0, -135.0, 180.0]
    # # print move_pose2
    # move_list = [move_pose1, move_pose2]
    # m_robot_master.move_pos_queue(move_list, vel=100, acc=200, syncType=1, time=1)
    # time.sleep(5)

    # indx = 1
    # while True:
    #     move_pose = [507.0, 0, 200, 35.434, -180, 35.434]
    #     m_robot_master.move_pos_robot(move_pose, vel=[800, 800], acc=[1000, 1000], syncType=1)
    #
    #     move_pose = [507.0, 50, 180, 120, -130, 120]
    #     m_robot_master.move_pos_robot(move_pose, vel=[100, 100], acc=[1000, 1000], syncType=1)
    #
    #     # time.sleep(2) # gripper operating time
    #
    #     move_pose = [307.0, 250, 180, 35.434, -180, 35.434]
    #     m_robot_master.move_pos_robot(move_pose, vel=[800, 800], acc=[1000, 1000], syncType=1)



    # m_robot_master.move_pos_robot(pose_init, vel=[100,50], acc= [50,50], syncType = 1)

    # pose = [450.0, 50, 440.5, 35.434, -180, 35.434]
    # pose_init = [207.0, 0, 440.5, 35.434, -180, 35.434]
    # while True:
    #     m_robot_master.move_pos_robot(pose_init, vel=[100,50], acc= [50,50], syncType = 1)
    #
    #     m_robot_master.gripper.grip()
    #     time.sleep(7)
    #
    #     f = open('/home/keti/DDI/start_capture.txt', 'w')
    #     f.close()
    #
    #     target_file = str(glob.glob('/home/keti/DDI/target_*.txt'))
    #     while(len(target_file) == 2):
    #         target_file = str(glob.glob('/home/keti/DDI/target_*.txt'))
    #
    #     target_file_split = target_file.split('.txt')[0].split('_')
    #     target = [float(target_file_split[1]), float(target_file_split[2]), float(target_file_split[3]), (180-float(target_file_split[4]))]
    #     print target
    #
    #     pose_goal = [target[0], target[1], pose_init[2], target[3]/2, pose_init[4], -target[3]/2]
    #     m_robot_master.move_pos_robot(pose_goal, vel=[100,100], acc= [50,50], syncType = 1)
    #
    #     m_robot_master.gripper.grip_release()
    #     time.sleep(7)
    #
    #     pose_goal = [target[0], target[1], pose_init[2] - (target[2] - pose_init[2] + 100) + 205, target[3]/2, pose_init[4], -target[3]/2]
    #     m_robot_master.move_pos_robot(pose_goal, vel=[50,100], acc= [20,50], syncType = 1)
    #
    #     m_robot_master.gripper.grip()
    #     time.sleep(7)
    #
    #     pose_goal = [target[0], target[1], pose_init[2], target[3]/2, pose_init[4], -target[3]/2]
    #     m_robot_master.move_pos_robot(pose_goal, vel=[50,100], acc=[20,50], syncType = 1)
    #
    #     os.remove(glob.glob('/home/keti/DDI/start_*.txt')[0])
    #     os.remove(glob.glob('/home/keti/DDI/target_*.txt')[0])
    #
    #     m_robot_master.gripper.grip_release()
    #     time.sleep(7)

    # m_robot_master.move_joint_robot([185.36,3.16,-125.24,180.27,60.8,0.48], vel=50, acc=10, syncType = 1)
    #
    # m_robot_master.gripper.move(30)
    # m_robot_master.gripper.move(30)
    # time.sleep(7)
    # m_robot_master.gripper.close()
    # time.sleep(5)
    #
    # m_robot_master.move_joint_robot([193.27,-21.51,-81.48,180.0,75.3,19.99], vel=50, acc=10, syncType=1)
    #
    # m_robot_master.gripper.open()
    # time.sleep(5)

    # m_robot_master.move_joint_robot([180.0, 0.0, -90.0, 180.0, 90.0, 0.0], vel=50, acc=10, syncType=1)

    # m_robot_master.gripper.grip(50)
    # time.sleep(10)

    # m_robot_master.gripper.grip()
    # time.sleep(10)

    # m_robot_master.move_joint_robot([185.0, 3.8, -92.0, 182.0, 90.0, 2.0], vel=50, acc=10, syncType=1)

    # m_robot_master.gripper.grip_release()
    # time.sleep(10)
    #
    # print m_robot_master.gripper.grip_get_pos()

    # m_robot_master.gripper.target(2000)
    # time.sleep(1)
    # m_robot_master.gripper.close('adjust')
    # time.sleep(3)
    # m_robot_master.gripper.close()
    # time.sleep(10)
    #
    # m_robot_master.gripper.open()
    # time.sleep(10)

    # m_robot_master.gripper.grip_release()
    # time.sleep(7)
    #
    # m_robot_master.gripper.grip()
    # time.sleep(7)

