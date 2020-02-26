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
        self.status = 0
        self.grip_distance = 0

        self.DataTransferOK = 0x1000
        self.MotorON = 0x0002
        self.MovementComplete = 0x0008
        self.InMotion = 0x0004
        self.HomingPositionOK = 0x0001

        for i in range(0, len(self.InputDataRegister)):
            self.InputData[i] = self.c.read_input_registers(self.InputDataRegister[i], 1, unit=16)

        self.cnt = 0

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

        self.OutputData[0] = 0x1
        self.OutputData[1] = 20480
        self.OutputData[2] = 100
        self.OutputData[3] = 95 * 256 + 95
        self.OutputData[4] = 100
        self.OutputData[5] = 200
        self.OutputData[6] = 300
        self.OutputData[7] = 4000

        self.t1 = threading.Thread(target=self.getStatus)
        self.t1.daemon = True
        self.t1.start()

    def getStatus(self):
        while True:
            self.cnt += 1
            if (self.cnt % 10) == 0:
                for i in range(0, len(self.InputDataRegister)):
                    self.InputData[i] = self.c.read_input_registers(self.InputDataRegister[i], 1, unit=16)
                print('[{0}] State word : {1}, Diagnosis : {2}, Position : {3}'.format(time.time(), self.InputData[0].registers[0], hex(self.InputData[1].registers[0]), self.InputData[2].registers[0]))
                msg_status = self.InputData[0].registers[0]
                self.grip_distance = self.InputData[2].registers[0]

                msg_diagnosis = self.InputData[1].registers[0]
                self.status = msg_diagnosis
                if msg_diagnosis != 0x0000 and self.init_complete:
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

    def grip(self, work_position=4000):
        self.OutputData[0] = 0x1
        self.OutputData[2] = 100
        self.OutputData[4] = 100
        self.OutputData[5] = 200
        self.OutputData[6] = 300
        self.OutputData[7] = work_position
        self.comm()

        self.OutputData[0] = 0x200  # move to work position
        self.comm()
        self.wait_move()

        print 'Gripper move to work position({0})'.format(work_position)

    def grip_release(self, base_position=100):
        if base_position != 100:
            self.OutputData[0] = 0x1
            self.OutputData[2] = base_position
            self.OutputData[4] = base_position
            self.OutputData[5] = base_position+100
            self.OutputData[6] = base_position+200
            self.OutputData[7] = 4000
            self.comm()

        self.OutputData[0] = 0x100  # move to base position
        self.comm()
        self.wait_move()

        print 'Gripper move to base position({0})'.format(base_position)

    def grip_get_pos(self):
        return self.grip_distance

    def grip_get_success(self):
        return self.status

if __name__ == '__main__':
    gripper = KetiZimmer()
    gripper.init()

    while True:
        try:
            gripper.grip_release()
            gripper.grip(1800)
            gripper.grip_release()
            gripper.grip()
        except KeyboardInterrupt:
            break
    print 'good bye'
