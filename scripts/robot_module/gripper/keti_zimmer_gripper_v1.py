#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pymodbus.client.sync import ModbusTcpClient
import time

# Output data word 0 - 0x0801 (ControlWord)
# Output data word 1 - 0x0802 (DeviceMode, Workpiece No)
# Output data word 2 - 0x0803 (Reserve, PositionTolerance)
# Output data word 3 - 0x0804 (GripForce, DriveVelocity)
# Output data word 4 - 0x0805 (BasePosition)
# Output data word 5 - 0x0806 (ShiftPosition)
# Output data word 5 - 0x0807 (TeachPosition)
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

    def init(self):
        self.OutputData[0] = 1
        self.OutputData[1] = 768
        for i in range(2, len(self.OutputDataRegister)):
            self.OutputData[i] = 0

        for i in range(0, len(self.OutputDataRegister)):
            self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])

        time.sleep(0.05)

        self.OutputData[0] = 0
        self.OutputData[1] = 768
        for i in range(2, len(self.OutputDataRegister)):
            self.OutputData[i] = 0

        for i in range(0, len(self.OutputDataRegister)):
            self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])

        time.sleep(0.05)

        self.target(4000, base_position=100)

        time.sleep(1)
        self.OutputData[0] = 256
        for i in range(0, len(self.OutputDataRegister)):
            self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])

        if self.print_flag is True:
            print 'Gripper Init'

    def target(self, position, mode='close', base_position=-1):
        if base_position != 100:
            base_position = self.status()

        self.OutputData[0] = 1
        self.OutputData[1] = 20480
        self.OutputData[3] = 2580
        self.OutputData[4] = self.OutputData[7] if mode == 'adjust' else base_position
        self.OutputData[5] = self.OutputData[4] + 200 if mode == 'adjust' else position - 500
        self.OutputData[6] = self.OutputData[5] + 100
        self.OutputData[7] = position
        self.OutputData[2] = position - self.OutputData[5]
        if self.print_flag is True and mode == 'adjust':
            print 'Gripper Target Setting Finished'

        for i in range(0, len(self.OutputDataRegister)):
            self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])

    def close(self, mode='close'):
        self.OutputData[0] = 512
        for i in range(0, len(self.OutputDataRegister)):
            self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])

        if mode == 'adjust':
            self.target(4000, 'adjust')
            time.sleep(1)

            self.OutputData[0] = 256
            for i in range(0, len(self.OutputDataRegister)):
                self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])

        if self.print_flag is True:
            print 'Gripper Close' if mode == 'close' else 'Gripper Close to Adjust position'

    def open(self):
        self.target(4000, base_position=100)
        time.sleep(1)

        self.OutputData[0] = 256
        for i in range(0, len(self.OutputDataRegister)):
            self.c.write_registers(self.OutputDataRegister[i], self.OutputData[i])

        if self.print_flag is True:
            print 'Gripper Open'

    def status(self):
        for i in range(0, len(self.InputDataRegister)):
            self.InputData[i] = self.c.read_input_registers(self.InputDataRegister[i], 1, unit=16)

        return self.InputData[2].registers[0]

if __name__ == '__main__':
    gripper = KetiZimmer()

    gripper.init()
    time.sleep(10)

    gripper.target(2000)
    time.sleep(5)

    gripper.close('adjust')
    time.sleep(5)

    gripper.close()
    time.sleep(10)

    gripper.open()
    time.sleep(5)

    print gripper.status()

# if __name__ == '__main__':
#     OutputDataRegister = [0x0801, 0x0802, 0x0803, 0x0804, 0x0805, 0x0806, 0x0807, 0x0808]
#     OutputData = [0, 0, 0, 0, 0, 0, 0, 0]
#     InputDataRegister = [0x0002, 0x0003, 0x0004]
#     InputData = [0, 0, 0]
#
#     IP_ADDRESS = '192.168.0.253'
#     SERVER_PORT = 502
#
#     c = ModbusTcpClient(host=IP_ADDRESS, port=SERVER_PORT)
#
#     print 'Gripper Init'
#     OutputData[0] = 1
#     OutputData[1] = 768
#     for i in range(2, len(OutputDataRegister)):
#         OutputData[i] = 0
#
#     for i in range(0, len(OutputDataRegister)):
#         c.write_registers(OutputDataRegister[i], (OutputData[i]))
#
#     time.sleep(1)
#
#     OutputData[0] = 0
#     OutputData[1] = 768
#     for i in range(2, len(OutputDataRegister)):
#         OutputData[i] = 0
#
#     for i in range(0, len(OutputDataRegister)):
#         c.write_registers(OutputDataRegister[i], (OutputData[i]))
#
#     time.sleep(1)
#
#     print 'Gripper Target Setting'
#     position = 2000
#     OutputData[0] = 1
#     OutputData[1] = 20480
#     OutputData[3] = 2580
#     OutputData[4] = 100
#     OutputData[5] = position - 500
#     OutputData[6] = OutputData[5] + 100
#     OutputData[7] = position
#     OutputData[2] = position - OutputData[5]
#
#     for i in range(0, len(OutputDataRegister)):
#         c.write_registers(OutputDataRegister[i], (OutputData[i]))
#
#     time.sleep(5)
#
#     print 'Gripper Adjust Close'
#     OutputData[0] = 512
#     for i in range(0, len(OutputDataRegister)):
#         c.write_registers(OutputDataRegister[i], (OutputData[i]))
#
#     time.sleep(5)
#
#     print 'Gripper Target Setting'
#     position = 4000
#     OutputData[0] = 1
#     OutputData[1] = 20480
#     OutputData[3] = 2580
#     OutputData[4] = OutputData[7]
#     OutputData[5] = OutputData[4] + 200
#     OutputData[6] = OutputData[5] + 100
#     OutputData[7] = position
#     OutputData[2] = position - OutputData[5]
#
#     for i in range(0, len(OutputDataRegister)):
#         c.write_registers(OutputDataRegister[i], (OutputData[i]))
#
#     time.sleep(5)
#
#     OutputData[0] = 256
#     for i in range(0, len(OutputDataRegister)):
#         c.write_registers(OutputDataRegister[i], (OutputData[i]))
#
#     time.sleep(5)
#
#     print 'Gripper Close'
#     OutputData[0] = 512
#     for i in range(0, len(OutputDataRegister)):
#         c.write_registers(OutputDataRegister[i], (OutputData[i]))
#
#     time.sleep(5)
#
#     position = 4000
#     OutputData[0] = 1
#     OutputData[1] = 20480
#     OutputData[3] = 2580
#     OutputData[4] = 100
#     OutputData[5] = position - 500
#     OutputData[6] = OutputData[5] + 100
#     OutputData[7] = position
#     OutputData[2] = position - OutputData[5]
#
#     for i in range(0, len(OutputDataRegister)):
#         c.write_registers(OutputDataRegister[i], (OutputData[i]))
#
#     time.sleep(5)
#
#     print 'Gripper Open'
#     OutputData[0] = 256
#     for i in range(0, len(OutputDataRegister)):
#         c.write_registers(OutputDataRegister[i], (OutputData[i]))
#
#     time.sleep(5)
#
#     for i in range(0, len(InputDataRegister)):
#         InputData[i] = c.read_input_registers(InputDataRegister[i], 1, unit=16)
#         print InputData[i].registers[0]
#
#     time.sleep(0.5)
