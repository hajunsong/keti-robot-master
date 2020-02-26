#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time, datetime
import sys, select, termios, tty
import math

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"/home/hajun/catkin_ws/src/doosan-robot/common/imp")) ) # get import pass : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "a0509"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

rot1 = []
rot2 = []
rot3 = []
pos_x = 0
pos_y = 0
pos_z = 0

def shutdown():
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"

    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb(msg):
    global rot1, rot2, rot3, pos_x, pos_y, pos_z
    msgRobotState_cb.count += 1

    rot1 = msg.rotation_matrix[0].data
    rot2 = msg.rotation_matrix[1].data
    rot3 = msg.rotation_matrix[2].data
    pos_x = msg.current_posx[0]
    pos_y = msg.current_posx[1]
    pos_z = msg.current_posx[2]

    R1 = []
    R2 = []
    R3 = []
    # sys.stdout.write("  rotation_matrix       : ")
    # for i in range(0, 3):
    #     sys.stdout.write("dim : [%d]" % i)
    #     sys.stdout.write("  [ ")
    #     for j in range(0, 3):
    #         sys.stdout.write("%f " % msg.rotation_matrix[i].data[j])
    #     sys.stdout.write("] ")

    R1 = msg.rotation_matrix[0].data
    R2 = msg.rotation_matrix[1].data
    R3 = msg.rotation_matrix[2].data

    phi = math.atan2(R3[1], R3[0])
    psi = math.atan2(R2[2], -R1[2])
    theta = math.atan2(R2[2], R3[2]*math.sin(psi))
    #
    # print '\n' + str(msg.current_posx[3]) + ', ' + str(msg.current_posx[4]) + ' ,' + str(msg.current_posx[5])
    # print '\n' + str(math.degrees(phi)) + ', ' + str(math.degrees(theta)) + ' ,' + str(math.degrees(psi))


msgRobotState_cb.count = 0


def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)


def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":
    # global pos_x, pos_y, pos_z, rot1, rot2, rot3

    rospy.init_node('dsr_service_motion_simple_keyop_py')
    rospy.on_shutdown(shutdown)

    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True
    t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           

    velx=[50, 50]
    accx=[100, 100]

    p1 = posj(180.0, 0.0, -90.0, 180.0, 90.0, 0.0)
    movej(p1, vel=20, acc=20, time=2)
    time.sleep(1)

    move_pose = [207.0, 0, 440.5, 35.434, -180, 35.434]
    movel(move_pose, velx, accx, time=2)
    time.sleep(1)

    interval_pos = 1
    interval_ori = 0.1

    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w':
            data = '[' + str(datetime.datetime.now()) + ']'
            data += 'Position := ' + str(pos_x) + ',' + str(pos_y) + ',' + str(pos_z)
            data += ',Rotation := ' + str(rot1[0]) + ',' + str(rot1[1]) + ',' + str(rot1[2])
            data += ',' + str(rot2[0]) + ',' + str(rot2[1]) + ',' + str(rot2[2])
            data += ',' + str(rot3[0]) + ',' + str(rot3[1]) + ',' + str(rot3[2]) + '\n'
            print data

            f = open('/home/keti/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/log.txt', 'a')
            f.write(data)
            time.sleep(2)
            f.close()

        elif key == 'a':
            move_pose[0] += interval_pos
        elif key == 's':
            move_pose[1] += interval_pos
        elif key == 'd':
            move_pose[2] += interval_pos
        elif key == 'f':
            move_pose[3] += interval_ori
        elif key == 'g':
            move_pose[4] += interval_ori
        elif key == 'h':
            move_pose[5] += interval_ori

        elif key == 'z':
            move_pose[0] -= interval_pos
        elif key == 'x':
            move_pose[1] -= interval_pos
        elif key == 'c':
            move_pose[2] -= interval_pos
        elif key == 'v':
            move_pose[3] -= interval_ori
        elif key == 'b':
            move_pose[4] -= interval_ori
        elif key == 'n':
            move_pose[5] -= interval_ori

        elif key == 'o':
            interval_pos += 1
            interval_ori += 0.1
        elif key == 'p':
            interval_pos -= 1
            interval_ori -= 0.1
            if interval_pos <= 1:
                interval_pos = 1
            if interval_ori <= 0.1:
                interval_ori = 0.1

        elif key == 'l':
            amovej(p1, vel=20, acc=20, time=2)
            time.sleep(1)

            move_pose = [207.0, 0, 440.5, 35.434, -180, 35.434]
            amovel(move_pose, velx, accx, time=2)
            time.sleep(1)

        elif key == 'q':
            break

        elif key == ' ':
            amovel(move_pose, velx, accx, time=2)

        print (move_pose, interval_pos, interval_ori)

    print 'good bye!'
