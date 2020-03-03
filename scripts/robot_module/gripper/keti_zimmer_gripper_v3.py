#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pymodbus.client.sync import ModbusTcpClient
import time
import threading

# Output data word 0 - 0x0801 (ControlWord)
# Output data word 1 - 0x0802 (DeviceMode, Workpiece No)
# Output data word 2 - 0x0803 (Reserve, PositionTolerance)
# Output data word 3 - 0x0804 (GripForce, DriveVelocity)
# Output data word 4 - 0x0805 (BasePosition)
# Output data word 5 - 0x0806 (ShiftPosition)
# Output data word 6 - 0x0807 (TeachPosition)
# Output data word 7 - 0x0808 (WorkPosition)

# Input data word 0 - 0x0002 (StatusWord)
# Input data word 1 - 0x0003 (Diagnosis)
# Input data word 2 - 0x0004 (ActualPosition)


class KetiZimmer():
    def __init__(self):
        self.IP_ADDRESS = '192.168.0.253'
        self.SERVER_PORT = 502

        self.c = ModbusTcpClient(host=self.IP_ADDRESS, port=self.SERVER_PORT)

        self.OutputDataRegister = [0x0801, 0x0802, 0x0803, 0x0804, 0x0805, 0x0806, 0x0807, 0x0808]
        self.OutputData = [0, 0, 0, 0, 0, 0, 0, 0]
        self.InputDataRegister = [0x0002, 0x0003, 0x0004]
        self.InputData = [0, 0, 0]
        self.print_flag = True
        self.init_complete = False
        self.status = False
        self.grip_distance = 0

        self.DataTransferOK = 0x1000
        self.MotorON = 0x0002
        self.MovementComplete = 0x0008
        self.InMotion = 0x0004
        self.HomingPositionOK = 0x0001

        for i in range(0, len(self.InputDataRegister)):
            self.InputData[i] = self.c.read_input_registers(self.InputDataRegister[i], 1, unit=16)

    def init(self):
        self.OutputData[0] = 0x01
        self.OutputData[1] = 0x0A * 0x100 + 0x00  # Homing Command
        for i in range(0, len(self.OutputDataRegister)):
            self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])
        time.sleep(0.5)
        while True:
            for i in range(0, len(self.InputDataRegister)):
                self.InputData[i] = self.c.read_input_registers(self.InputDataRegister[i], 1, unit=16)

            if self.InputData[0].registers[0] & self.HomingPositionOK == self.HomingPositionOK:
                break
        print 'Homing Position OK'

        self.OutputData[0] = 0x01  # Data Transfer
        self.OutputData[1] = 0x05 * 0x100 + 0x00  # Motor OFF
        for i in range(0, len(self.OutputDataRegister)):
            self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])
        time.sleep(0.1)

        self.OutputData[0] = 0x01  # Data Transfer
        self.OutputData[1] = 0x03 * 0x100 + 0x00  # Motor ON
        for i in range(0, len(self.OutputDataRegister)):
            self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])
        time.sleep(0.1)

        while True:
            for i in range(0, len(self.InputDataRegister)):
                self.InputData[i] = self.c.read_input_registers(self.InputDataRegister[i], 1, unit=16)

            if self.InputData[0].registers[0] & self.MotorON == self.MotorON:
                break

        if self.print_flag is True:
            print 'Gripper Init'
        self.init_complete = True

    def comm(self):
        time.sleep(0.1)
        for i in range(0, len(self.OutputDataRegister)):
            self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])

        if self.OutputData[0] == 1:
            while True:
                for i in range(0, len(self.OutputDataRegister)):
                    self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])

                for i in range(0, len(self.InputDataRegister)):
                    self.InputData[i] = self.c.read_input_registers(self.InputDataRegister[i], 1, unit=16)

                if self.InputData[0].registers[0] & self.DataTransferOK == self.DataTransferOK:
                    break

        if self.print_flag is True:
            print 'Comm Complete'

        return True

    def wait_move(self):
        time.sleep(0.5)
        while True:
            for i in range(0, len(self.InputDataRegister)):
                self.InputData[i] = self.c.read_input_registers(self.InputDataRegister[i], 1, unit=16)

            if self.InputData[0].registers[0] & self.MovementComplete == self.MovementComplete:
                break
        print 'Movement Complete'

    def grip(self, pos=4000):

        gripper.OutputData[0] = 0x1
        gripper.OutputData[2] = 100
        gripper.OutputData[4] = 100
        gripper.OutputData[5] = 200
        gripper.OutputData[6] = 300
        gripper.OutputData[7] = pos
        gripper.comm()

        gripper.OutputData[0] = 0x200  # move to work position
        gripper.comm()
        gripper.wait_move()

        print 'gripper move to work position({0})'.format(pos)

    def grip_release(self):
        gripper.OutputData[0] = 0x1
        gripper.OutputData[1] = 20480
        gripper.OutputData[2] = 100
        gripper.OutputData[3] = 95 * 256 + 95
        gripper.OutputData[4] = 100
        gripper.OutputData[5] = 200
        gripper.OutputData[6] = 300
        gripper.OutputData[7] = 4000
        gripper.comm()

        gripper.OutputData[0] = 0x100  # move to base position
        gripper.comm()
        gripper.wait_move()

        print 'gripper move to base position'

    def grip_get_pos(self):
        return gripper.grip_distance

    # def target(self, position, mode='close', base_position=-1):
    #     if base_position != 600:
    #         base_position = self.status()
    #
    #     self.OutputData[0] = 1
    #     self.OutputData[1] = 20480
    #     self.OutputData[3] = 100*256 + 100
    #     self.OutputData[4] = self.OutputData[7] - 2000 if mode == 'adjust' else base_position
    #     self.OutputData[5] = self.OutputData[4] + 200 if mode == 'adjust' else position - 1500
    #     self.OutputData[6] = self.OutputData[5] + 100
    #     self.OutputData[7] = position
    #     self.OutputData[2] = position - self.OutputData[5]
    #     if self.print_flag is True and mode == 'adjust':
    #         print 'Gripper Target Setting Finished'
    #
    #     self.comm()
    #
    # def close(self, mode='close'):
    #     self.OutputData[0] = 512
    #     for i in range(0, len(self.OutputDataRegister)):
    #         self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])
    #
    #     if mode == 'adjust':
    #         self.target(4000, 'adjust')
    #         time.sleep(1)
    #
    #         self.OutputData[0] = 256
    #         for i in range(0, len(self.OutputDataRegister)):
    #             self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])
    #
    #     if self.print_flag is True:
    #         print 'Gripper Close' if mode == 'close' else 'Gripper Close to Adjust position'
    #
    # def open(self):
    #     self.target(4000, base_position=600)
    #     time.sleep(1)
    #
    #     self.OutputData[0] = 256
    #     for i in range(0, len(self.OutputDataRegister)):
    #         self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])
    #
    #     if self.print_flag is True:
    #         print 'Gripper Open'
    #
    # def status(self):
    #     for i in range(0, len(self.InputDataRegister)):
    #         self.InputData[i] = self.c.read_input_registers(self.InputDataRegister[i], 1, unit=16)
    #
    #     return self.InputData[2].registers[0]
    #
    # def grip(self, pos=-1):
    #     grip_between_distance = 76 # [mm]
    #     tooltip_size = 2500 # [0.01 mm]
    #
    #     if pos == -1:
    #         position = 4000
    #     else:
    #         position = 4000 - tooltip_size - int(str(grip_between_distance/2 - pos/2))
    #         # print str(grip_between_distance/2 - pos/2)
    #
    #     self.target(position)
    #     time.sleep(1)
    #
    #     if pos == -1:
    #         self.close()
    #     else:
    #         self.close('adjust')
    #
    #     if self.print_flag is True:
    #         print 'Gripper Close to ' + str(position)
    #
    # def grip_release(self):
    #     self.open()
    #
    # def grip_get_pos(self):
    #     position = self.status()
    #     # pos =
    #     return position
    #
    # def grip_get_success(self):
    #     return -1

def getStatus(gripper):
    while True:
        for i in range(0, len(gripper.InputDataRegister)):
            gripper.InputData[i] = gripper.c.read_input_registers(gripper.InputDataRegister[i], 1, unit=16)
        print('[{0}] State word : {1}, Diagnosis : {2}, Position : {3}'.format(time.time(), gripper.InputData[0].registers[0], hex(gripper.InputData[1].registers[0]), gripper.InputData[2].registers[0]))
        msg_status = gripper.InputData[1].registers[0]
        gripper.grip_distance = gripper.InputData[2].registers[0]

        msg_diagnosis = gripper.InputData[1].registers[0]
        if msg_diagnosis != 0x0000 and gripper.init_complete:
            if msg_diagnosis == 0x0001:
                print '[GRIPPER WARNING]Motor is switched off'
            elif msg_diagnosis == 0x0002:
                print '[GRIPPER WARNING]Gripper performs a reference run'
            elif msg_diagnosis == 0x0003:
                print '[GRIPPER WARNING]System boots up'
            elif msg_diagnosis == 0x0100:
                print '[GRIPPER WARNING]Actuator voltage is too low'
            elif msg_diagnosis == 0x0101:
                print '[GRIPPER WARNING]Max. permitted temperature exceeded'
            elif msg_diagnosis == 0x0102:
                print '[GRIPPER WARNING]Max. permitted temperature undershot'
            elif msg_diagnosis == 0x0200:
                print '[GRIPPER WARNING]IO-Link communication faulty'
            elif msg_diagnosis == 0x0201:
                print '[GRIPPER WARNING]SPI communication faulty'
            elif msg_diagnosis == 0x0202:
                print '[GRIPPER WARNING]CAN communication faulty'
            elif msg_diagnosis == 0x0204:
                print '[GRIPPER WARNING]STP safety circuit interrupted'
            elif msg_diagnosis == 0x0300:
                print '[GRIPPER WARNING]"ControlWord" not available'
            elif msg_diagnosis == 0x0301:
                print '[GRIPPER WARNING]Positions not available'
            elif msg_diagnosis == 0x0302:
                print '[GRIPPER WARNING]"GripForce" not available'
            elif msg_diagnosis == 0x0303:
                print '[GRIPPER WARNING]"SpeedValue" not available'
            elif msg_diagnosis == 0x0304:
                print '[GRIPPER WARNING]"TeachingTolerance" not available'
            elif msg_diagnosis == 0x0305:
                print '[GRIPPER WARNING]Position measuring system not referenced'
                gripper.status = False
            elif msg_diagnosis == 0x0306:
                print '[GRIPPER WARNING]"DeviceMode" not available'
            elif msg_diagnosis == 0x0307:
                print '[GRIPPER WARNING]Movement order cannot be carried out'
            elif msg_diagnosis == 0x0308:
                print '[GRIPPER WARNING]"WorkpieceNo." not available'
            elif msg_diagnosis == 0x313:
                print '[GRIPPER WARNING]Calculated "ShiftPosition" exceeded'
            elif msg_diagnosis == 0x0400:
                print '[GRIPPER WARNING]Difficulty of movement'
            elif msg_diagnosis == 0x0401:
                print '[GRIPPER WARNING]Current limit exceeded'
            elif msg_diagnosis == 0x0402:
                print '[GRIPPER WARNING]Jam'
            elif msg_diagnosis == 0x0406:
                print '[GRIPPER WARNING]System/internal error'
        else:
            gripper.status = True
        time.sleep(1)

if __name__ == '__main__':
    gripper = KetiZimmer()

    gripper.init()

    t1 = threading.Thread(target=getStatus, args=(gripper,))
    t1.daemon = True
    t1.start()

    while True:
        try:
            gripper.grip_release()
            gripper.grip(1800)
            gripper.grip_release()
            gripper.grip()
            # gripper.OutputData[0] = 0x1
            # gripper.OutputData[1] = 20480
            # gripper.OutputData[2] = 100
            # gripper.OutputData[3] = 95 * 256 + 95
            # gripper.OutputData[4] = 100
            # gripper.OutputData[5] = 200
            # gripper.OutputData[6] = 300
            # gripper.OutputData[7] = 4000
            # gripper.comm()
            #
            # gripper.OutputData[0] = 0x100  # move to base position
            # gripper.comm()
            # gripper.wait_move()
            #
            # print 'gripper move to base position'
            #
            # gripper.OutputData[0] = 0x1
            # gripper.OutputData[2] = 100
            # gripper.OutputData[4] = 100
            # gripper.OutputData[5] = 200
            # gripper.OutputData[6] = 300
            # gripper.OutputData[7] = 1800
            # gripper.comm()
            #
            # gripper.OutputData[0] = 0x200  # move to work position
            # gripper.comm()
            # gripper.wait_move()
            #
            # print 'gripper move to new work position'
            #
            # # gripper.OutputData[0] = 0x4  # move to work position
            # # gripper.comm()
            #
            # # print 'gripper direction reset'
            #
            # gripper.OutputData[0] = 0x100  # move to base position
            # gripper.comm()
            # gripper.wait_move()
            #
            # print 'gripper move to base position'
            #
            # gripper.OutputData[0] = 0x1
            # gripper.OutputData[2] = 100
            # gripper.OutputData[4] = 100
            # gripper.OutputData[5] = 200
            # gripper.OutputData[6] = 300
            # gripper.OutputData[7] = 4000
            # gripper.comm()
            #
            # gripper.OutputData[0] = 0x200  # move to work position
            # gripper.comm()
            # gripper.wait_move()

            # time.sleep(5)

            # print 'gripper move to work position'
        except KeyboardInterrupt:
            break
    print 'good bye'
