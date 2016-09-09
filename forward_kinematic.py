"""
Compute forward kinematic and print foot coordinates.
"""
from __future__ import print_function
import sys
import can
import time
import numpy as np
import blmc.motor_data as md


BITRATE = 1e6


def tf01(th):
    Cth = np.cos(th)
    Sth = np.sin(th)
    return np.matrix([[Cth,  Sth, 0.1*Cth],
                      [Sth, -Cth, 0.1*Sth],
                      [  0,    0,       1]])


def tf12(th):
    Cth = np.cos(th)
    Sth = np.sin(th)
    return np.matrix([[Cth, -Sth, 0.1*Cth],
                      [Sth,  Cth, 0.1*Sth],
                      [  0,    0,       1]])


def mrev_to_rad(mrev):
    return mrev * 2*np.pi


if __name__ == "__main__":
    mtr_data = md.MotorData()
    bus = can.interface.Bus(bitrate=BITRATE)

    #canhndlr = md.MessageHandler()
    #canhndlr.set_id_handler(md.ArbitrationIds.status, mtr_data.set_status)
    #canhndlr.set_id_handler(md.ArbitrationIds.current, mtr_data.set_current)
    #canhndlr.set_id_handler(md.ArbitrationIds.position, mtr_data.set_position)
    #canhndlr.set_id_handler(md.ArbitrationIds.velocity, mtr_data.set_velocity)
    #canhndlr.set_id_handler(md.ArbitrationIds.adc6, adc.set_values)

    last_print = 0
    foot_2 = np.matrix([0, 0, 1]).transpose()

    # wait for messages and update data
    for msg in bus:
        #canhndlr.handle_msg(msg.arbitration_id, msg.data)
        t = time.clock()
        if msg.arbitration_id == md.ArbitrationIds.position:
            mtr_data.set_position(msg)
            th1 = mrev_to_rad(mtr_data.mtr1.position.value) / 3
            th2 = mrev_to_rad(mtr_data.mtr2.position.value) / 3
            foot_0 = tf01(th1) * tf12(th2) * foot_2
            # to cm
            foot_0 *= 100

            if last_print < t - 1:
                print()
                print("th1: {:.2f}\t th2: {:.2f}".format(th1*180/np.pi, th2*180/np.pi))
                print("x: {:.2f}\t y: {:.2f}".format(foot_0[0, 0], foot_0[1, 0]))
                last_print = t
