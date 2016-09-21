"""
Compute forward kinematic and print foot coordinates.
"""
from __future__ import print_function
import sys
import can
import time
import numpy as np
import blmc.motor_data as md
from blmc.kinematic_leg1 import *
import blmc.can_helper as ch


BITRATE = 1e6


if __name__ == "__main__":
    mtr_data = md.MotorData()
    bus = can.interface.Bus(bitrate=BITRATE)

    last_print = 0
    foot_2 = np.matrix([0, 0, 1]).transpose()

    ch.send_command(bus, ch.Command.send_position, 1)

    # wait for messages and update data
    for msg in bus:
        #canhndlr.handle_msg(msg.arbitration_id, msg.data)
        t = time.clock()
        if msg.arbitration_id == md.ArbitrationIds.position:
            mtr_data.set_position(msg)
            th1 = mrev_to_rad(mtr_data.mtr1.position.value) / 3
            th2 = mrev_to_rad(mtr_data.mtr2.position.value) / 3
            foot_0 = tf01(th1) * tf12(th2) * foot_2
            foot_0 = np.asarray(foot_0.transpose())[0, :2]

            if last_print < t - 1:
                print()
                print("th1: {:.2f}\t th2: {:.2f}".format(th1*180/np.pi,
                                                         th2*180/np.pi))
                print("x: {:.2f}\t y: {:.2f}".format(foot_0[0]*100,
                                                     foot_0[1]*100))
                (ith1, ith2) = inverse_kinematics(foot_0[0], foot_0[1])
                print("Ith1: {:.2f}\t Ith2: {:.2f}".format(ith1*180/np.pi,
                                                           ith2*180/np.pi))
                if is_pose_safe(mtr_data.mtr1.position.value,
                                mtr_data.mtr2.position.value,
                                foot_0):
                    print("Okay.")
                else:
                    print("DANGER!")
                last_print = t
