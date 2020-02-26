#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pymodbus.client.sync import ModbusTcpClient
import struct
import time


class KetiSchunk:

    def __init__(self):
        self.IP_ADDRESS = '192.168.0.253'
        self.SERVER_PORT = 502

        self.c = ModbusTcpClient(host=self.IP_ADDRESS, port=self.SERVER_PORT)

        self.c.write_registers(0x0801, 0x0)

        time.sleep(1)

        self.a = 0x10000000
        self.cmd_select = 0x8100
        self.while_cnt = 0
        self.user_input = '0'
        self.ack_cmd = 0
        self.trigger_flag = 0
        self.target_position = 0.1
        self.IEEE754_hex_high = 0
        self.IEEE754_hex_low = 0

        self.c.write_registers(0x0804, 0x0000)
        self.c.write_registers(0x0803, 0x3f00)
        self.c.write_registers(0x0804, 0x0000)

        self.TargetPos = 0
        self.TargetTrq = 0

        self.position = 0
        self.torque = 0

        self.rr0 = self.c.read_input_registers(0, 8, unit=1)
        self.rr1 = self.c.read_input_registers(0x0801, 8, unit=1)

        self.status_3bit = 0

    def float_bin(self, number, places=3):
        whole, dec = str(number).split(".")
        whole = int(whole)
        dec = int(dec)
        res = bin(whole).lstrip("0b") + "."

        for x in range(places):
            a = float(self.decimal_converter(dec))
            whole, dec = str(a * 2.0).split(".")
            dec = int(dec)
            res += whole
        return res

    def decimal_converter(self, num):
        while num >= 1:
            num /= 10.0
        return num

    def IEEE754(self, n):
        # identifying whether the number
        # is positive or negative
        sign = 0
        if n < 0:
            sign = 1
            n = n * (-1)
        p = 30

        # convert float to binary
        dec = self.float_bin(n, places=p)

        # separate the decimal part
        # and the whole number part
        whole, dec = str(dec).split(".")
        whole = int(whole)

        # calculating the exponent(E)
        exponent = len(str(whole)) - 1
        exponent_bits = 127 + exponent

        # converting the exponent from
        # decimal to binary
        exponent_bits = bin(exponent_bits).lstrip("0b")

        # finding the mantissa
        mantissa = str(whole)[1:exponent + 1]
        mantissa = mantissa + dec
        mantissa = mantissa[0:23]

        # the IEEE754 notation in binary
        final = str(sign) + str(exponent_bits) + mantissa

        # convert the binary to hexadecimal
        ln = (len(final))
        b = int(final, 2)
        hstr = '%0*X' % ((len(final) + 3) // 4, int(final, 2))

        # return the answer to the driver code
        return hstr

    def status(self, print_flag=False):
        time.sleep(0.5)
        self.rr0 = self.c.read_input_registers(0, 8, unit=1)
        self.rr1 = self.c.read_input_registers(0x0801, 8, unit=1)
        d = self.rr0.registers[4] | self.rr0.registers[3] << 16
        if print_flag == True:
            Tmpstr = "Read = 1[" + hex(self.rr0.registers[1])
            Tmpstr += "] 2[" + hex(self.rr0.registers[2])
            Tmpstr += "] 3[" + hex(self.rr0.registers[2])
            Tmpstr += "] 4[" + hex(self.rr0.registers[2])
            Tmpstr += "]"
            print(Tmpstr)
            print("Process cmd : {0:2d}    Blocked    : {0:2d}   EndStop : {0:2d}".format((self.rr0.registers[1] >> 15 & 1),
                                                                                          (self.rr0.registers[1] >> 14 & 1),
                                                                                          (self.rr0.registers[1] >> 13 & 1)))
            print("Success :     {0:2d}    Referecnce : {0:2d}  ".format((self.rr0.registers[1] >> 12 & 1),
                                                                         (self.rr0.registers[1] >> 11 & 1)))

        status_3bit = self.rr0.registers[1] >> 8 & 0x7
        if print_flag == True:
            if status_3bit == 0x00:
                print("Status : Error")
            elif status_3bit == 0x01:
                print("Status : Out of specification")
            elif status_3bit == 0x02:
                print("Status : Maintenance required")
            elif status_3bit == 0x03:
                print("Status : Ready for operation")
            print("Position : {0:8f}".format(struct.unpack('f', struct.pack('I', d))[0]))
            Tmpstr = "Wtite = 1[" + hex(self.rr1.registers[1])
            Tmpstr += "] 2[" + hex(self.rr1.registers[2])
            Tmpstr += "] 3[" + hex(self.rr1.registers[2])
            Tmpstr += "] 4[" + hex(self.rr1.registers[2])
            Tmpstr += "]"
            print(Tmpstr)
            print("-")

        return format(struct.unpack('f', struct.pack('I', d))[0])

    def comm(self):
        while True:
            time.sleep(0.5)
            self.rr0 = self.c.read_input_registers(0, 8, unit=1)
            self.rr1 = self.c.read_input_registers(0x0801, 8, unit=1)
            d = self.rr0.registers[4] | self.rr0.registers[3] << 16
            Tmpstr = "Read = 1[" + hex(self.rr0.registers[1])
            Tmpstr += "] 2[" + hex(self.rr0.registers[2])
            Tmpstr += "] 3[" + hex(self.rr0.registers[2])
            Tmpstr += "] 4[" + hex(self.rr0.registers[2])
            Tmpstr += "]"

            self.status_3bit = self.rr0.registers[1] >> 8 & 0x7

            if self.rr0.registers[1] & 0x8000 == 0x8000:
                ack_cmd = self.cmd_select & 0x7fff
                self.c.write_registers(0x0801, ack_cmd)
                self.trigger_flag = 1

            if self.trigger_flag == 1 and self.rr0.registers[1] & 0x8000 == 0:
                if self.cmd_select == 0x8700:
                    position = self.IEEE754(float(str(self.position)))
                    self.TargetPos = int(position, 16)
                    self.TargetTrq = self.torque

                elif self.cmd_select & 0x8000 == 0x8000:
                    self.c.write_registers(0x801, self.cmd_select)
                    self.c.write_registers(0x802, (self.TargetTrq & 0x03) << 8)  # 75% 힘으로
                    self.c.write_registers(0x803, (self.TargetPos >> 16) & 0xffff)
                    self.c.write_registers(0x804, (self.TargetPos) & 0xffff)
                    self.trigger_flag = 0

                break

    def init(self):
        self.c.write_registers(0x0801, 0x8100) # acknowleding
        time.sleep(0.5)
        self.cmd_select = 0x8200 # referencing
        self.comm()
        if self.status_3bit == 0x03:
            print 'Gripper Initialization Complete'
        else:
            print 'Gripper Initialization Error'

    def open(self):
        self.cmd_select = 0x8300
        self.comm()
        if self.status_3bit == 0x03:
            print 'Gripper Open'
        else:
            print 'Gripper Error(Open)'

    def close(self):
        self.cmd_select = 0x8400
        self.comm()
        if self.status_3bit == 0x03:
            print 'Gripper Close'
        else:
            print 'Gripper Error(Close)'

    def move(self, position, torque=0):
        self.position = position
        self.torque = torque
        self.cmd_select = 0x8700
        self.comm()
        if self.status_3bit == 0x03:
            print 'Gripper Target Position Apply -- ' + str(self.position)
        else:
            print 'Gripper Error(Target Position Setting)'
        self.cmd_select = 0x8500
        self.comm()
        if self.status_3bit == 0x03:
            print 'Gripper Move Target Position'
        else:
            print 'Gripper Error(Move Target Position)'

if __name__ == '__main__' :
    gripper = KetiSchunk()

    gripper.init()
    time.sleep(5)

    gripper.move(20)
    time.sleep(7)

    gripper.close()
    time.sleep(5)

    gripper.open()
    time.sleep(5)

    print 'exit'

