"""
Small working example on how to access CAN bus.
"""
from __future__ import print_function
import os
import time
import can
import signal
import sys
import traceback
import numpy as np
from blmc.optoforce import *
from blmc.motor_data import *
from blmc.conversion import *
from blmc.controllers import PositionController
from blmc.can_helper import *

BITRATE = 1e6


def get_position_reference(t):
    # range of motor 1: +/- 1.0
    # range of motor 2: +/- 1.1
    # start both at 0 and run with same frequency (i.e. they always are at zero
    # at the same time)

    range1 = 0.4
    range2 = 0.8
    freq = 0.7

    t_scale = 2.0 * np.pi * freq
    t = t * t_scale

    s1 = range1 * np.sin(t)
    s2 = range2 * np.sin(t)

    return (s1, s2)


def emergency_break(bus):
    send_mtr_current(bus, 0, 0)
    send_msg(bus, msg_disable_system)
    print("EMERGENCY STOP")
    sys.exit(0)


if __name__ == "__main__":
    #if len(sys.argv) != 8:
    #   print("Usage: {} Kp1 Ki1 Kd1 Kp2 Ki2 Kd2".format(sys.argv[0]))
    #   sys.exit(1)


    if len(sys.argv) == 7:
        Kp1 = float(sys.argv[1])
        Ki1 = float(sys.argv[2])
        Kd1 = float(sys.argv[3])
        Kp2 = float(sys.argv[4])
        Ki2 = float(sys.argv[5])
        Kd2 = float(sys.argv[6])
    else:
        print("Use default controller values")
        (Kp1, Ki1, Kd1) = (16, 0, 0.12)
        (Kp2, Ki2, Kd2) = (7, 0, 0.07)

    bus = can.interface.Bus(bitrate=BITRATE)

    # setup sigint handler to disable motor on CTRL+C
    def sigint_handler(signal, frame):
            print('Stop motor and shut down.')
            send_mtr_current(bus, 0, 0)
            send_msg(bus, msg_disable_motor1)
            send_msg(bus, msg_disable_motor2)
            sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)


    mtr_data = MotorData()
    adc = AdcResult()
    pos_ctrl1 = PositionController(Kp1, Ki1, Kd1)
    pos_ctrl2 = PositionController(Kp2, Ki2, Kd2)
    t_offset = None

    print("Setup controller 1 with Kp = {}, Ki = {}, Kd = {}".format(
        Kp1, Ki1, Kd1))
    print("Setup controller 2 with Kp = {}, Ki = {}, Kd = {}".format(
        Kp2, Ki2, Kd2))
    print()

    start_system(bus, mtr_data)

    raw_input("Move leg to zero position and press enter to start movement.")

    # Make sure we have the latest position data
    update_position(bus, mtr_data, 0.5)

    def on_position_msg(msg):
        global t_offset

        mtr_data.set_position(msg)

        if ((abs(mtr_data.mtr1.position.value) > 1.1)
                or (abs(mtr_data.mtr2.position.value) > 1.1)):
            emergency_break(bus)

        print(mtr_data.to_string())
        #print(adc.to_string())

        t = time.clock()
        if t_offset is None:
            t_offset = t
            # TODO check if position is close to zero at start
        t = t - t_offset
        # wait a moment before starting the sinus by using an additional
        # offset
        start_up_offset = 3
        if t > start_up_offset:
            t = t - start_up_offset

            (posref1, posref2) = get_position_reference(t)
        else:
            posref1 = 0
            posref2 = 0

        pos_ctrl1.update_data(mtr_data.mtr1)
        pos_ctrl1.run(posref1)
        pos_ctrl2.update_data(mtr_data.mtr2)
        pos_ctrl2.run(posref2)

        send_mtr_current(bus, pos_ctrl1.iqref, pos_ctrl2.iqref)

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
            send_msg(bus, msg_disable_system)
            break
