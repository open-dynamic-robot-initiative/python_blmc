"""
Controll the position of the foot, using sliders for x and y.
"""
from __future__ import print_function
import os
import time
import can
import signal
import sys
import traceback
from blmc.motor_data import *
from blmc.conversion import *
from blmc.controllers import PositionController
from blmc.can_helper import *
import blmc.kinematic_leg1 as kin

BITRATE = 1e6


if __name__ == "__main__":
    if len(sys.argv) != 8:
        print("Usage: {} goal_pos Kp1 Ki1 Kd1 Kp2 Ki2 Kd2".format(sys.argv[0]))
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
            send_mtr_current(bus, 0, 0)
            send_msg(bus, msg_disable_motor1)
            send_msg(bus, msg_disable_motor2)
            sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)


    print("Setup controller 1 with Kp = {}, Ki = {}, Kd = {}".format(
        Kp1, Ki1, Kd1))
    print("Setup controller 2 with Kp = {}, Ki = {}, Kd = {}".format(
        Kp2, Ki2, Kd2))
    print("Goal position: {}".format(goal_pos))
    vctrl1 = PositionController(Kp1, Ki1, Kd1)
    vctrl2 = PositionController(Kp2, Ki2, Kd2)

    print("Enable system...")
    send_msg(bus, msg_ensable_system)

    print("Enable motors...")
    send_mtr_current(bus, 0, 0) # start with zero
    send_msg(bus, msg_enable_motor1)
    send_msg(bus, msg_enable_motor2)

    # wait a moment for the initial messages to be handled
    time.sleep(0.2)

    first_x = None

    def on_position_msg(msg):
        global first_x

        mtr_data.set_position(msg)

        print(mtr_data.to_string())
        print(adc.to_string())

        mpos1 = mtr_data.mtr1.position.value
        mpos2 = mtr_data.mtr2.position.value

        foot_pos = kin.foot_position(mpos1, mpos2)

        if not kin.is_pose_safe(mpos1, mpos2, foot_pos):
            raise RuntimeError("EMERGENCY BREAK")

        if first_x is None:
            first_x = foot_pos[0]

        goal_x = first_x - adc.a / 10.0
        goal_y = adc.b / 10.0

        (goal_mpos1, goal_mpos2) = kin.inverse_kinematics_mrev(goal_x, goal_y)

        if mtr_data.status.mtr1_ready:
            vctrl1.update_data(mtr_data.mtr1)
            vctrl1.run(goal_mpos1)

        if mtr_data.status.mtr2_ready:
            vctrl2.update_data(mtr_data.mtr2)
            vctrl2.run(goal_mpos2)

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
            send_msg(bus, msg_disable_system)
            break
