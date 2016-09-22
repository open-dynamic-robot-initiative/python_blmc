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


class PositionMaster:

    def get_goal_pos(self):
        return np.zeros(2)

class PositionBySlider(PositionMaster):

    def __init__(self, adc):
        self._adc = adc
        self.max_x = 0.195

    def get_goal_pos(self):
        foot_goal = np.empty(2)
        foot_goal[0] = self.max_x - adc.a / 5.0
        foot_goal[1] = (adc.b - 0.5) / 3.0
        return foot_goal

class JumpTrajectory(PositionMaster):

    def __init__(self, ground_contact_state):
        self._gcs = ground_contact_state
        self._last_on_ground = self._gcs.on_ground
        self._t = time.clock()
        self._kneel_down_duration = 1

        self._y = 0.015
        self._x_air = 0.13
        self._x_kneel = 0.08
        self._x_stretched = 0.197

    def get_goal_pos(self):
        if self._gcs.on_ground:
            t = time.clock()
            if not self._last_on_ground:
                # Just landed. Reset timer.
                self._t = t

            t = t - self._t
            if t < self._kneel_down_duration:
                x = self._x_air - ((self._x_air - self._x_kneel) *
                        t / self._kneel_down_duration)
            else:
                x = self._x_stretched

            goal = np.array([x, self._y])

        else:
            # We are the air. Go to landing pose immediately.
            goal = np.array([self._x_air, self._y])

        self._last_on_ground = self._gcs.on_ground
        return goal


class GroundContactState:

    def __init__(self):
        self.on_ground = False  # assume we start in mid-air

    def update_state(self, foot_force):
        self.on_ground = foot_force[0] > 0.7


if __name__ == "__main__":
    # ground contact
    (Kp1, Ki1, Kd1) = (50, 0, 0.15)
    (Kp2, Ki2, Kd2) = (20, 0, 0.1)
    # air
    (Kp1_a, Ki1_a, Kd1_a) = (15, 0, 0.29)
    (Kp2_a, Ki2_a, Kd2_a) = (6, 0, 0.23)


    mtr_data = MotorData()
    adc = AdcResult()
    bus = can.interface.Bus(bitrate=BITRATE)

    ground_state = GroundContactState()

    #position_master = PositionBySlider(adc)
    position_master = JumpTrajectory(ground_state)

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
    vctrl1 = PositionController(Kp1, Ki1, Kd1)
    vctrl2 = PositionController(Kp2, Ki2, Kd2)

    start_system(bus, mtr_data)

    raw_input("Press Enter to start foot position control")

    # Make sure we have the latest position data
    update_position(bus, mtr_data, 0.5)

    position_ticks = 0

    def on_position_msg(msg):
        global position_ticks

        mtr_data.set_position(msg)

        position_ticks += 1
        if position_ticks < 2:
            return
        position_ticks = 0

        #print(mtr_data.to_string())
        #print(adc.to_string())

        current_mpos = np.array([mtr_data.mtr1.position.value,
                                 mtr_data.mtr2.position.value])

        foot_pos = kin.foot_position(current_mpos[0], current_mpos[1])

        if not kin.is_pose_safe(current_mpos[0], current_mpos[1], foot_pos):
            raise RuntimeError("EMERGENCY BREAK")

        foot_goal = position_master.get_goal_pos()

        foot_pos_error = np.linalg.norm(foot_pos - foot_goal)
        print("(x,y) = ({:.3f}, {:.3f}) ~ ({:.3f}, {:.3f}), err = {}".format(
            foot_pos[0], foot_pos[1], foot_goal[0], foot_goal[1],
            foot_pos_error))

        goal_mpos = np.asarray(kin.inverse_kinematics_mrev(
            foot_goal[0], foot_goal[1]))

        #print("(th1, th2) = ({:.3f}, {:.3f}) ~ ({:.3f}, {:.3f})".format(
        #    current_mpos[0], current_mpos[1], goal_mpos[0], goal_mpos[1]))

        force = kin.foot_force(
                mtr_data.mtr1.position.value, mtr_data.mtr2.position.value,
                mtr_data.mtr1.current.value, mtr_data.mtr2.current.value)
        #print("force: {}".format(force))

        ground_state.update_state(force)
        if ground_state.on_ground:
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
            stop_system(bus)
            break
