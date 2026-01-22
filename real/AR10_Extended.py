"""
Active8 Robots AR10 hand extended base program
author: Callum Pritchard, Active8 Robots

Description: this is intended to be a hgiher-level module for manipulating the hand, without having to use the lower-level functions
             building upon the base module, shows how AR10.py can be easily used to build up joints, digits and a hand
             motion_manager shows how to better manage the move commands, by keeping track of targets and updating all channels at once


install pyserial (not serial!)
ensure both this file and AR10.py are available

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

from real.AR10 import AR10
import time

class hand:
    def __init__(self):
        c_port = input("what serial port is hand mounted on? (eg. COM3 for windows, ttyacm3 for linux)")
        self.AR10 = AR10(c_port)

        #class to manage motion of hand, ensures asynchronously-sent move commands do not halt one another
        class motion_manager:
            def __init__(self, outer):
                self.defaultSpeed = 100
                self.defaultAcceleration = 0
                self.pending_moves = [8000,8000,8000,8000,8000,8000,8000,8000,8000,8000]
                self.moving = False;
                self.hand = outer
                for j in self.hand.joint_array:
                    time.sleep(0.3) #leave time before updating each channel - if done without a delay this will cause commands to be lost
                    j.set_speed(self.defaultSpeed)
                    j.set_acceleration(self.defaultAcceleration)
            def submit_move(self, channel, target):
                self.pending_moves[channel-10] = target
                self.try_move()
            def try_move(self):
                self.hand.AR10.set_all_channel_targets(*self.pending_moves)
            def home_hand(self):
                self.pending_moves = [8000,8000,8000,8000,8000,8000,8000,8000,8000,8000]
                self.try_move()
            def multi_move(self, pos_array):
                self.pending_moves = pos_array
                self.try_move()
            def get_current_targets(self):
                return self.pending_moves

        #class for digits on hand
        class digit:
            def __init__(self, outer, digit_index, friendly_name, base_joint, curl_joint):
                self.digit_index = digit_index
                self.friendly_name = friendly_name
                self.base_joint = base_joint
                self.curl_joint = curl_joint
                self.hand = outer
            def close(self):
                self.base_joint.move(3000)
                self.curl_joint.move(3000)

        #class for joints
        class joint:
            def __init__(self, outer, joint_index, channel_index, digit_index, friendly_name):
                self.joint_index = joint_index
                self.channel_index = channel_index
                self.digit_index = digit_index
                self.friendly_name = friendly_name
                self.hand = outer
            #move the target for this joint in a safe manner
            def move(self, target):
                self.hand.motion_manager.submit_move(self.channel_index, target)
            #this will move ONLY this channels target - if anything else is set too quickly after (eg: for loop) then commands will be lost in an unpredictable fashion
            def move_own_channel(self, target):
                self.hand.AR10.set_channel_target(self.channel_index, target)
            def set_speed(self, speed):
                self.hand.AR10.set_channel_speed(self.channel_index, speed)
            def set_acceleration(self, acceleration):
                self.hand.AR10.set_channel_acceleration(self.channel_index, acceleration)

        self.joint_array = [
            joint(self, 0, 10, 0, "thumb base"),
            joint(self, 1, 11, 0, "thumb curl"),
            joint(self, 2, 18, 1, "index base"),
            joint(self, 3, 19, 1, "index curl"),
            joint(self, 4, 16, 2, "middle base"),
            joint(self, 5, 17, 2, "middle curl"),
            joint(self, 6, 14, 3, "ring base"),
            joint(self, 7, 15, 3, "ring curl"),
            joint(self, 8, 12, 4, "little base"),
            joint(self, 9, 13, 4, "little curl")
        ]
        self.digit_array = [
            digit(self, 0, "thumb", self.joint_array[0], self.joint_array[1]),
            digit(self, 1, "index", self.joint_array[2], self.joint_array[3]),
            digit(self, 2, "middle", self.joint_array[4], self.joint_array[5]),
            digit(self, 3, "ring", self.joint_array[6], self.joint_array[7]),
            digit(self, 4, "little", self.joint_array[8], self.joint_array[9])
        ]
        self.motion_manager = motion_manager(self)

    def close(self):
        self.AR10.close()

    def open(self):
        open_targets = [self.AR10.target_max] * len(self.joint_array)
        self.motion_manager.multi_move(open_targets)
