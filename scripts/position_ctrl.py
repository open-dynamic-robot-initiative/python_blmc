#!/usr/bin/env python
"""
Small working example for a position controller

Use sliders on ADC A6 and B6 to set the position reference for the two motors.
"""

import os
import time
import can
import signal
import sys
import traceback
from blmc.optoforce import *
from blmc.motor_data import *
from blmc.conversion import *
from blmc.controllers import PositionController
from blmc.can_helper import *

BITRATE = 1e6


if __name__ == "__main__":
    if len(sys.argv) != 8:
        print("Usage: {} max_pos Kp1 Ki1 Kd1 Kp2 Ki2 Kd2".format(sys.argv[0]))
        sys.exit(1)

    goal_pos = float(sys.argv[1])
    Kp1 = float(sys.argv[2])
    Ki1 = float(sys.argv[3])
    Kd1 = float(sys.argv[4])
    Kp2 = float(sys.argv[5])
    Ki2 = float(sys.argv[6])
    Kd2 = float(sys.argv[7])

    mtr_data = MotorData()
    adc = AdcResult()
    bus = can.interface.Bus(bitrate=BITRATE)

    # setup sigint handler to disable motor on CTRL+C
    def sigint_handler(signal, frame):
            print('Stop motor and shut down.')
            stop_system(bus)
            sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)

    print("Setup controller 1 with Kp = {}, Ki = {}, Kd = {}".format(
        Kp1, Ki1, Kd1))
    print("Setup controller 2 with Kp = {}, Ki = {}, Kd = {}".format(
        Kp2, Ki2, Kd2))
    print("Goal position: {}".format(goal_pos))
    print()
    vctrl1 = PositionController(Kp1, Ki1, Kd1)
    vctrl2 = PositionController(Kp2, Ki2, Kd2)

    start_system(bus, mtr_data, init_position=False)

    input("Move both sliders to zero. Then press Enter to start motor"
              " position control")

    # Make sure we have the latest position data
    update_position(bus, mtr_data, 0.5)

    def on_position_msg(msg):
        mtr_data.set_position(msg)

        print(mtr_data.to_string())
        print(adc.to_string())

        if mtr_data.status.mtr1_ready:
            vctrl1.update_data(mtr_data.mtr1)
            vctrl1.run(goal_pos * adc.a)

        if mtr_data.status.mtr2_ready:
            vctrl2.update_data(mtr_data.mtr2)
            vctrl2.run(goal_pos * adc.b)

        send_mtr_current(bus, vctrl1.iqref, vctrl2.iqref)
        print()

    msg_handler = MessageHandler()
    msg_handler.set_id_handler(ArbitrationIds.status, mtr_data.set_status)
    msg_handler.set_id_handler(ArbitrationIds.current, mtr_data.set_current)
    msg_handler.set_id_handler(ArbitrationIds.position, on_position_msg)
    msg_handler.set_id_handler(ArbitrationIds.velocity, mtr_data.set_velocity)
    msg_handler.set_id_handler(ArbitrationIds.adc6, adc.set_values)

    # wait for messages and update data
    for msg in bus:
        try:
            msg_handler.handle_msg(msg)
        except:
            print("\n\n=========== ERROR ============")
            print(traceback.format_exc())
            stop_system(bus)
            break
