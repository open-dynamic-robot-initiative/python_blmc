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
import numpy as np
from blmc.motor_data import *
from blmc.conversion import *
from blmc.controllers import PositionController
from blmc.can_helper import *
import blmc.kinematic_leg1 as kin

BITRATE = 1e6


def comp_foot_force(q1, q2, iq1, iq2):
    l1 = 0.1
    l2 = 0.1
    t1 = iq1 * 0.018 * 3
    t2 = iq2 * 0.018 * 3
    sin = np.sin
    cos = np.cos

    fx = -(l1*l2*t1*sin(q2) + (l1*cos(q1) + l2*cos(q1 - q2))*(l2*t1*sin(q1 -
        q2) + t2*(l1*sin(q1) + l2*sin(q1 - q2))))/(l1*l2*(l1*sin(q1) +
            l2*sin(q1 - q2))*sin(q2))
    fy = -(l1*t2*sin(q1) + l2*t1*sin(q1 - q2) + l2*t2*sin(q1 -
        q2))/(l1*l2*sin(q2))

    return np.array((fx, fy))


if __name__ == "__main__":
    # ground contact
    (Kp1, Ki1, Kd1) = (50, 0, 0.15)
    (Kp2, Ki2, Kd2) = (20, 0, 0.1)
    # air
    (Kp1_a, Ki1_a, Kd1_a) = (10, 0, 0.23)
    (Kp2_a, Ki2_a, Kd2_a) = (6, 0, 0.23)


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
    vctrl1 = PositionController(Kp1, Ki1, Kd1)
    vctrl2 = PositionController(Kp2, Ki2, Kd2)

    print("Enable system...")
    send_msg(bus, msg_ensable_system)

    print("Enable motors...")
    send_mtr_current(bus, 0, 0) # start with zero
    send_msg(bus, msg_enable_motor1)
    send_msg(bus, msg_enable_motor2)

    # Wait till the motors are ready
    printed_align_msg = False
    for msg in bus:
        if msg.arbitration_id == ArbitrationIds.status:
            mtr_data.set_status(msg)
            if mtr_data.status.mtr1_ready and mtr_data.status.mtr2_ready:
                break
            elif not printed_align_msg:
                print("Motors are aligned. Please wait...")
                printed_align_msg = True

    # Initialize leg position
    init_position_offset(bus, mtr_data)

    raw_input("Press Enter to start foot position control")

    # Update position
    start = time.clock()
    for msg in bus:
        if msg.arbitration_id == ArbitrationIds.position:
            if start < time.clock() - 1:
                mtr_data.set_position(msg)
                break

    first_x = None
    position_ticks = 0

    def on_position_msg(msg):
        global first_x, position_ticks

        mtr_data.set_position(msg)

        position_ticks += 1
        if position_ticks < 2:
            return
        position_ticks = 0

        print(mtr_data.to_string())
        #print(adc.to_string())

        current_mpos = np.array([mtr_data.mtr1.position.value,
                                 mtr_data.mtr2.position.value])

        foot_pos = kin.foot_position(current_mpos[0], current_mpos[1])

        if not kin.is_pose_safe(current_mpos[0], current_mpos[1], foot_pos):
            raise RuntimeError("EMERGENCY BREAK")

        if first_x is None:
            first_x = foot_pos[0]

        foot_goal = np.empty(2)
        foot_goal[0] = first_x - adc.a / 5.0
        foot_goal[1] = (adc.b - 0.5) / 3.0

        foot_pos_error = np.linalg.norm(foot_pos - foot_goal)
        #print("(x,y) = ({:.3f}, {:.3f}) ~ ({:.3f}, {:.3f}), err = {}".format(
        #    foot_pos[0], foot_pos[1], foot_goal[0], foot_goal[1],
        #    foot_pos_error))

        goal_mpos = np.asarray(kin.inverse_kinematics_mrev(
            foot_goal[0], foot_goal[1]))

        #print("(th1, th2) = ({:.3f}, {:.3f}) ~ ({:.3f}, {:.3f})".format(
        #    current_mpos[0], current_mpos[1], goal_mpos[0], goal_mpos[1]))

        q1 = kin.mrev_to_rad(mtr_data.mtr1.position.value) / 3
        q2 = kin.mrev_to_rad(mtr_data.mtr2.position.value) / 3
        force = comp_foot_force(q1, q2,
                mtr_data.mtr1.current.value, mtr_data.mtr2.current.value)
        force_N = np.linalg.norm(force)
        print("force: {}".format(force))
        #print("force: {} N".format(force_N))

        if force[0] > 0.7:
            print("Ground")
            vctrl1.update_gains(Kp1, Ki1, Kd1)
            vctrl2.update_gains(Kp2, Ki2, Kd2)
        else:
            print("Air")
            vctrl1.update_gains(Kp1_a, Ki1_a, Kd1_a)
            vctrl2.update_gains(Kp2_a, Ki2_a, Kd2_a)

        if mtr_data.status.mtr1_ready:
            vctrl1.update_data(mtr_data.mtr1)
            vctrl1.run(goal_mpos[0], verbose=False)

        if mtr_data.status.mtr2_ready:
            vctrl2.update_data(mtr_data.mtr2)
            vctrl2.run(goal_mpos[1], verbose=False)

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
