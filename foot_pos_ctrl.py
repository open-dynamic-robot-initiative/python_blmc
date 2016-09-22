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


class LinearTrajectory:
    """Generate a linear trajectory."""

    def __init__(self, start, end, speed):
        """Initialize.

        Parameter
        =========
        start : array
            Start position
        end : array
            End positon
        speed : float
            Movement speed in m/s.
        """
        self.start = start
        self.end = end
        self.speed = speed
        self._t_start = None

        se = self.end - self.start
        self._dist_start_end = np.linalg.norm(se)
        self.step = se / self._dist_start_end * speed

    def next_step(self):
        """Get the next step of the trajectory or None if goal is reached."""
        t = time.clock()
        if self._t_start is None:
            self._t_start = t
        t -= self._t_start

        # Vector from start to end
        se = self.end - self.start
        if self._dist_start_end == 0:
            # seems like start == end. We are already at the goal :)
            return None

        # to unit length
        se_t = self.step * t

        if np.linalg.norm(se_t) < self._dist_start_end:
            return self.start + se_t
        else:
            # reached end
            return None


class PositionMaster:

    def get_goal_pos(self, current_pos):
        raise NotImplementedError()


class PositionBySlider(PositionMaster):

    def __init__(self, adc):
        self._start_up = True
        self._adc = adc
        self._start_traj = None
        self._slider_pos = np.zeros(2)
        self.max_x = 0.195

    def get_goal_pos(self, current_pos):
        self._slider_pos[0] = self.max_x - adc.a / 5.0
        self._slider_pos[1] = (adc.b - 0.5) / 3.0

        if self._start_up:
            return self._do_start_up(current_pos)
        else:
            return self._slider_pos

    def get_pid_gains(self):
        return ((30, 0, 0.15), (15, 0, 0.1))

    def _do_start_up(self, current_pos):
        print("*** STARTUP ***")
        if self._start_traj is None:
            self._start_traj = LinearTrajectory(current_pos, self._slider_pos,
                    0.03)
        goal = self._start_traj.next_step()
        if goal is None:
            goal = self._slider_pos
            self._start_up = False
        return goal


class State:
    START_UP = 1
    AIR = 2
    GROUND = 3


class JumpTrajectory(PositionMaster):

    def __init__(self, ground_contact_state):
        self._state = State.START_UP
        self._gcs = ground_contact_state
        self._kneel_down_traj = None
        self._start_traj = None

        self._kneel_down_speed = 0.1
        y = 0.015
        self._p_air = np.array([0.13, y])
        self._p_kneel = np.array([0.08, y])
        self._p_stretched = np.array([0.197, y])

    def get_goal_pos(self, current_pos):
        if self._state == State.START_UP:
            return self._do_start_up(current_pos)
        elif self._state == State.AIR:
            return self._do_air()
        else: # state == GROUND
            return self._do_ground()

    def get_pid_gains(self):
        if self._state == State.START_UP:
            return ((30, 0, 0.15), (15, 0, 0.1))
        elif self._state == State.AIR:
            return ((15, 0, 0.28), (6, 0, 0.23))
        else: # state == GROUND
            return ((50, 0, 0.15), (20, 0, 0.1))

    def _do_start_up(self, current_pos):
        print("*** STARTUP ***")
        if self._start_traj is None:
            self._start_traj = LinearTrajectory(
                    current_pos, self._p_air, 0.05)
        goal = self._start_traj.next_step()
        if goal is None:
            goal = self._p_air
            self._state = State.AIR
        return goal

    def _do_air(self):
        print("~~~ AIR ~~~")
        if self._gcs.on_ground:
            self._state = State.GROUND
            # Just landed. Set new kneel trajectory
            self._kneel_down_traj = LinearTrajectory(
                    self._p_air, self._p_kneel, self._kneel_down_speed)

        return self._p_air

    def _do_ground(self):
        print("___GROUND___")
        goal = self._kneel_down_traj.next_step()
        if goal is None:
            goal = self._p_stretched
        if not self._gcs.on_ground:
            self._state = State.AIR
        return goal


class GroundContactState:

    def __init__(self):
        self.on_ground = False  # assume we start in mid-air

    def update_state(self, foot_force):
        self.on_ground = foot_force[0] > 0.7


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ["slider", "jump"]:
        print("Usage: {} slider|jump".format(sys.argv[0]))
        sys.exit(0)


    mtr_data = MotorData()
    adc = AdcResult()
    bus = can.interface.Bus(bitrate=BITRATE)
    ground_state = GroundContactState()
    position_ticks = 0

    if sys.argv[1] == "slider":
        print("Use the sliders")
        print("===============\n")
        position_master = PositionBySlider(adc)
    elif sys.argv[1] == "jump":
        print("Let it jump!")
        print("============\n")
        position_master = JumpTrajectory(ground_state)
    else:
        raise ValueError("Invalid mode selection")

    # setup sigint handler to disable motor on CTRL+C
    def sigint_handler(signal, frame):
            print('Stop motor and shut down.')
            stop_system(bus)
            sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)

    pid_gains = position_master.get_pid_gains()
    vctrl1 = PositionController(*pid_gains[0])
    vctrl2 = PositionController(*pid_gains[1])

    start_system(bus, mtr_data)

    raw_input("Press Enter to start foot position control")

    # Make sure we have the latest position data
    update_position(bus, mtr_data, 0.5)


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

        force = kin.foot_force(
                mtr_data.mtr1.position.value, mtr_data.mtr2.position.value,
                mtr_data.mtr1.current.value, mtr_data.mtr2.current.value)
        #print("force: {}".format(force))
        ground_state.update_state(force)

        foot_goal = position_master.get_goal_pos(foot_pos)

        foot_pos_error = np.linalg.norm(foot_pos - foot_goal)
        print("(x,y) = ({:.3f}, {:.3f}) ~ ({:.3f}, {:.3f}), err = {}".format(
            foot_pos[0], foot_pos[1], foot_goal[0], foot_goal[1],
            foot_pos_error))

        goal_mpos = np.asarray(kin.inverse_kinematics_mrev(
            foot_goal[0], foot_goal[1]))

        #print("(th1, th2) = ({:.3f}, {:.3f}) ~ ({:.3f}, {:.3f})".format(
        #    current_mpos[0], current_mpos[1], goal_mpos[0], goal_mpos[1]))

        pid_gains = position_master.get_pid_gains()
        vctrl1.update_gains(*pid_gains[0])
        vctrl2.update_gains(*pid_gains[1])

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
