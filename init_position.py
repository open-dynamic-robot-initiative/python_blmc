"""
Demo on how to initialize the zero position at start up.
"""
from __future__ import print_function
import can
import time
import blmc.motor_data as md
import blmc.can_helper as ch
from blmc.helper import get_time


BITRATE = 1e6


if __name__ == "__main__":
    mtr_data = md.MotorData()
    bus = can.interface.Bus(bitrate=BITRATE)
    last_print = 0

    # We need the position data for this to work
    ch.send_command(bus, ch.Command.send_position, 1)

    # Initialize the position
    md.init_position_offset(bus, mtr_data)
    time.sleep(1)

    # Print position data to validate initialization
    for msg in bus:
        t = get_time()
        if msg.arbitration_id == md.ArbitrationIds.position:
            mtr_data.set_position(msg)

            if last_print < t - 1:
                last_print = t
                print("pos1: {}, pos2: {}".format(
                    mtr_data.mtr1.position.value,
                    mtr_data.mtr2.position.value))
