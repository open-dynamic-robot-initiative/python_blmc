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

    print("Move both joints in positive direction until stop. Then press enter")
    raw_input("")

    # Offsets between positive stop and zero for the current leg:
    stop_offsets = (-1.38418251276, -1.38845098019)
    zero_offsets = None

    start = time.clock()
    for msg in bus:
        if msg.arbitration_id == md.ArbitrationIds.position:
            mtr_data.set_position(msg)
            zero_offsets = (mtr_data.mtr1.position.value + stop_offsets[0],
                            mtr_data.mtr2.position.value + stop_offsets[1])
            if start < time.clock() - 1:
                print("Okay, now release the leg")
                break
    time.sleep(1)

    # wait for messages and update data
    for msg in bus:
        #canhndlr.handle_msg(msg.arbitration_id, msg.data)
        t = time.clock()
        if msg.arbitration_id == md.ArbitrationIds.position:
            mtr_data.set_position(msg)

            if last_print < t - 1:
                last_print = t
                print("pos1: {}, pos2: {}".format(
                    mtr_data.mtr1.position.value - zero_offsets[0],
                    mtr_data.mtr2.position.value - zero_offsets[1]))
