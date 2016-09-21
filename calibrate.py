"""
Calibrate the stop offsets for position initialization.
"""
from __future__ import print_function
import can
import time
import numpy as np
import cPickle as pickle
import blmc.motor_data as md
import blmc.can_helper as ch


BITRATE = 1e6


if __name__ == "__main__":
    mtr_data = md.MotorData()
    bus = can.interface.Bus(bitrate=BITRATE,
            can_filter=[{"can_id": md.ArbitrationIds.position,
                         "can_mask": 0xFFF}])
    last_print = 0

    ch.send_command(bus, ch.Command.send_position, 1)

    raw_input("Move both joints in positive direction until stop."
              " Then press enter.")
    start = time.clock()
    for msg in bus:
        if msg.arbitration_id == md.ArbitrationIds.position:
            if start < time.clock() - 1:
                mtr_data.set_position(msg)
                stop_position = (mtr_data.mtr1.position.value,
                                 mtr_data.mtr2.position.value)
                break


    raw_input("Now move the leg to the zero position (fully stretched,"
              " poiting downwards). Then press enter.")
    start = time.clock()
    for msg in bus:
        if msg.arbitration_id == md.ArbitrationIds.position:
            if start < time.clock() - 1:
                mtr_data.set_position(msg)
                zero_offsets = (mtr_data.mtr1.position.value,
                                mtr_data.mtr2.position.value)
                break

    stop_offsets = np.array(stop_position) - np.array(zero_offsets)

    print("Okay, now release the leg")
    print("stop_offset: {}".format(stop_offsets))
    print("zero_offset: {}".format(zero_offsets))

    print("Store calibration data to file 'calibration_data.p'")
    pickle.dump(stop_offsets, open("calibration_data.p", "wb"),
            pickle.HIGHEST_PROTOCOL)
    print("done")
