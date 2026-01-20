"""
Active8 Robots AR10 hand base program
author: Callum Pritchard, Active8 Robots

Description: base module for dealing with AR10 hand, Pololu Mini Maestro 24
             this can be used as reference to see what the various commands do, but does nothing if run standalone

install pyserial (not serial!)

REVIEW POLULU MAESTRO DOCUMENTATION FOR INFORMATION ON SENDING COMMANDS CORRECTLY

channel and joint numbering

____Desc____|_joint_|_digit_|_channel__
thumb, base |   0   |    0  |    10
thumb, curl |   1   |    0  |    11
little,base |   8   |    4  |    12
little,curl |   9   |    4  |    13
ring, base  |   6   |    3  |    14
ring, curl  |   7   |    3  |    15
middle,base |   4   |    2  |    16
middle,curl |   5   |    2  |    17
index, base |   2   |    1  |    18
index, curl |   3   |    1  |    19
"""

import serial

class AR10:
    def __init__(self, comPort):

        self.usb = serial.Serial(comPort, baudrate=9600)
        self.target_max = 7700
        self.target_min = 4200

    def clean_target(self, target):
        if target > self.target_max:
            target = self.target_max
        elif target < self.target_min:
            target = self.target_min
        return target

    #close the USB
    def close(self):
        self.usb.close()
        print("goodbye!")

    #send commands to polulu
    def send_command(self, *inputs):
        prelude = chr(0xaa) + chr(0xc)
        for i in inputs:
            prelude = prelude + i

        self.usb.write(prelude.encode())

    #set the speed of a channel (max rate at which the pulse-width changes) measured in units of 0.25us/10ms
    # 0 is unlimited
    #eg at speed 140, controller will take 100ms to change from 1000us to 1350us pulse width. this example ignores acceleration
    def set_channel_speed(self, channel, speed):
        if speed < 0:
            speed = 0
        lsb = speed & 0x7f           #7 bits for least significant BYTE
        msb = (speed >> 7) & 0x7f    #shift 7 for the 7 bits of MSB
        self.send_command(chr(0x07), chr(channel), chr(lsb), chr(msb));

    #similar to the speed of channels, but sets the rate of change the controller uses when adjusting pulse-width of a channel, both starting the move, and when slowing to meet the target
    #measures as a value 0-255 in units 0.25us/10ms/80ms
    #0 is unlimited
    def set_channel_acceleration(self, channel, acceleration):
        if acceleration < 0:
            acceleration = 0
        elif acceleration > 255:
            acceleration = 255
        lsb = acceleration & 0x7f           #7 bits for least significant BYTE
        msb = (acceleration >> 7) & 0x7f    #shift 7 for the 7 bits of MSB
        self.send_command(chr(0x09), chr(channel), chr(lsb), chr(msb));

    #set a target for a channel, cleaned by the clean_target function
    def set_channel_target(self, channel, target):
        t = self.clean_target(target)
        lsb = t & 0x7f                      #7 bits for least significant BYTE
        msb = (target >> 7) & 0x7f          #shift 7 for the 7 bits of MSB
        self.send_command(chr(0x04), chr(channel), chr(lsb), chr(msb))

    #set the target for every channel. provide a target for every channel
    def set_all_channel_targets(self, *targets):
        command_args = [chr(0x1f), chr(10), chr(10)]
        for t in targets:
            t = self.clean_target(t)
            command_args.append(chr(t & 0x7f)) #lsb for target
            command_args.append(chr((t >> 7) & 0x7f)) #msb for target
        self.send_command(*command_args) #this send command_args as seperate arguments

    #if speed is set below top speed (and isnt set to 0) this will return the actual servo get_position
    #otherwise this will simply be the last target set and therefor not reliable
    def get_channel_target(self, channel):
        self.send_command(chr(0x10), chr(channel))
        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return ((msb << 8) + lsb)

    #if all servo targets have been reached this returns True. this assumes speed is not 0 and is below servo max speed
    def get_hand_waiting(self):
        self.send_command(chr(0x13), chr(0x01))
        if self.usb.read() == chr(0):
            return True
        else:
            return False

    #get any errors reported by the Maestro controller
    def get_errors(self):
        self.send_command(chr(0x21))
        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return ((msb << 8) + lsb)
