#!/usr/bin/env python
"""
Compute forward kinematic and print foot coordinates.

First the joint positions are initialized, then the script starts to print
 * joint positions from board
 * foot position by computing forward kinematics
 * joint positions from inverse kinematics (based on foot position)
 * Whether or not the current pose is safe (= inside the allowed working range)
"""

import sys
import can
import numpy as np
import blmc.motor_data as md
import blmc.kinematic_leg1 as kin
import blmc.can_helper as ch
from blmc.helper import get_time


BITRATE = 1e6


if __name__ == "__main__":
    mtr_data = md.MotorData()
    bus = can.interface.Bus(bitrate=BITRATE)
    tf = kin.Transformer()
    last_print = 0

    ch.send_command(bus, ch.Command.send_position, 1)
    md.init_position_offset(bus, mtr_data)

    # wait for messages and update data
    for msg in bus:
        t = get_time()
        if msg.arbitration_id == md.ArbitrationIds.position:
            mtr_data.set_position(msg)
            tf.update_mtr_data(mtr_data)
            foot_0 = tf.foot_position()

            if last_print < t - 1:
                print()
                print("th1: {:.2f}\t th2: {:.2f}".format(tf.th1*180/np.pi,
                                                         tf.th2*180/np.pi))
                print("x: {:.2f}\t y: {:.2f}".format(foot_0[0]*100,
                                                     foot_0[1]*100))
                (ith1, ith2) = kin.inverse_kinematics(foot_0[0], foot_0[1])
                print("IK: th1: {:.2f}\t th2: {:.2f}".format(ith1*180/np.pi,
                                                           ith2*180/np.pi))
                if tf.is_pose_safe():
                    print("Pose is safe.")
                else:
                    print("Pose is UNSAFE!")
                last_print = t
