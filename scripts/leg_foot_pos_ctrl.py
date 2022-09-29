#!/usr/bin/env python
"""
Control the position of the foot.  There are two modes: "slider" and "jump".


Slider
------

When called with the argument "slider", the position of the foot can be
controlled via sliders.

The sliders have to be connected to the ADCs A6 and B6 of the board.  The
A-slider sets the vertical position of the foot, the B-slider the horizontal.
For the initial position (leg fully streched) set the A-slider to 0 and the
B-slider to half of its range.


Jump
----

When called with the argument "jump", the foot will move to a slightly crouched
position.  As soon as it touches the ground, the leg will slowly crouch down
and then immediately straighten the leg to jump.  This is repeated over and
over again.

Note that in this mode touchdown detection is done based on motor torques and
different gains for the PD controller are used depending on if the leg touches
the ground or not.
"""

import os
import can
import signal
import sys
import traceback
import numpy as np
from blmc.motor_data import *
from blmc.conversion import *
from blmc.controllers import PositionController
from blmc.can_helper import *
from blmc.helper import get_time
import blmc.kinematic_leg1 as kin
import blmc.optoforce as of

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
        t = get_time()
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

        self._kneel_down_speed = 0.05
        y = 0.02
        self._p_air = np.array([0.13, y])
        self._p_kneel = np.array([0.07, y])
        self._p_stretched = np.array([0.197, y])

    def get_goal_pos(self, current_pos):
        if self._state == State.START_UP:
            return self._do_start_up(current_pos)
        elif self._state == State.AIR:
            return self._do_air(current_pos)
        else: # state == GROUND
            return self._do_ground()

    def get_pid_gains(self):
        if self._state == State.START_UP:
            return ((30, 0, 0.15), (15, 0, 0.1))
        elif self._state == State.AIR:
            return ((15, 0, 0.3), (6, 0, 0.23))
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

    def _do_air(self, current_pos):
        print("~~~ AIR ~~~")
        if self._gcs.on_ground:
            self._state = State.GROUND
            # Just landed. Set new kneel trajectory
            # Adjust y value
            start = np.array([self._p_air[0], current_pos[1]])
            self._p_kneel[1] = current_pos[1]
            self._p_stretched[1] = current_pos[1]
            self._kneel_down_traj = LinearTrajectory(
                    start, self._p_kneel, self._kneel_down_speed)

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
        if self.on_ground:
            self.on_ground = foot_force[0] > 0.4
        else:
            self.on_ground = foot_force[0] > 0.7


class PositionLogger:
    time = []
    foot_pos = []
    foot_pos_ref = []
    mtr1_pos = []
    mtr1_pos_ref = []
    mtr2_pos = []
    mtr2_pos_ref = []

    def log_data(self, foot_pos, mtr_data, foot_ref, mtr_ref):
        self.time.append(mtr_data.mtr1.position.timestamp)

        self.foot_pos.append(np.array(foot_pos))
        self.foot_pos_ref.append(np.array(foot_ref))

        self.mtr1_pos.append(mtr_data.mtr1.position.value)
        self.mtr1_pos_ref.append(mtr_ref[0])

        self.mtr2_pos.append(mtr_data.mtr2.position.value)
        self.mtr2_pos_ref.append(mtr_ref[1])

    def __del__(self):
        with open("logged_data.txt", "w") as fh:
            fh.write("# time foot_x foot_y foot_x_ref foot_y_ref mtr1_pos"
                     " mtr1_pos_ref mtr2_pos mtr2_pos_ref\n")
            for i in range(len(self.time)):
                fh.write(" ".join([str(x) for x in [
                    self.time[i],
                    self.foot_pos[i][0],
                    self.foot_pos[i][1],
                    self.foot_pos_ref[i][0],
                    self.foot_pos_ref[i][1],
                    self.mtr1_pos[i],
                    self.mtr1_pos_ref[i],
                    self.mtr2_pos[i],
                    self.mtr2_pos_ref[i]
                    ]]) + "\n")


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ["slider", "jump"]:
        print("Usage: {} slider|jump".format(sys.argv[0]))
        sys.exit(0)


    mtr_data = MotorData()
    adc = AdcResult()
    tf = kin.Transformer()
    optoforce = of.OptoForcePacketReceiver()
    bus = can.interface.Bus(bitrate=BITRATE)
    ground_state = GroundContactState()
    logger = PositionLogger()
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

    #print("Initialize OptoForce...")
    ofconf = of.OptoForceConfig()
    ofconf.zero()
    ofconf.set_sample_frequency(
           of.OptoForceConfig.SampleFreq.Hz_10)
    ofconf.send_via_can(bus, ArbitrationIds.optoforce_recv)

    start_system(bus, mtr_data)

    input("Press Enter to start foot position control")

    # Make sure we have the latest position data
    update_position(bus, mtr_data, 0.5)


    def on_position_msg(msg):
        global position_ticks

        loop_start = get_time()

        mtr_data.set_position(msg)

        # reduce frequency of the controller (only run every second time)
        position_ticks += 1
        if position_ticks < 2:
            return
        position_ticks = 0

        tf.update_mtr_data(mtr_data)

        print(mtr_data.to_string())
        #print(adc.to_string())

        foot_pos = tf.foot_position()

        if not tf.is_pose_safe():
            raise RuntimeError("EMERGENCY BREAK")

        force = tf.foot_force(
                mtr_data.mtr1.current.value, mtr_data.mtr2.current.value)
        print("force (motor): {}".format(force))
        if optoforce.data:
            of_force_s = np.array([optoforce.data.fx_N,
                                   optoforce.data.fy_N,
                                   optoforce.data.fz_N])
            of_force_f = tf.transform_optoforce_to_base(of_force_s)
            #print("force (opto s):  {}".format(of_force_s))
            print("force (opto f):  {}".format(of_force_f[:2]))
        ground_state.update_state(force)

        foot_goal = position_master.get_goal_pos(foot_pos)

        foot_pos_error = np.linalg.norm(foot_pos - foot_goal)
        print("(x,y) = ({:.3f}, {:.3f}) ~ ({:.3f}, {:.3f}), err = {}".format(
            foot_pos[0], foot_pos[1], foot_goal[0], foot_goal[1],
            foot_pos_error))

        goal_mpos = np.asarray(kin.inverse_kinematics_mrev(
            foot_goal[0], foot_goal[1]))

        #print("(th1, th2) = ({:.3f}, {:.3f}) ~ ({:.3f}, {:.3f})".format(
        #    tf.th1, tf.th2, goal_mpos[0], goal_mpos[1]))

        pid_gains = position_master.get_pid_gains()
        vctrl1.update_gains(*pid_gains[0])
        vctrl2.update_gains(*pid_gains[1])

        if mtr_data.status.mtr1_ready:
            vctrl1.update_data(mtr_data.mtr1)
            vctrl1.run(goal_mpos[0], verbose=False)

        if mtr_data.status.mtr2_ready:
            vctrl2.update_data(mtr_data.mtr2)
            vctrl2.run(goal_mpos[1], verbose=False)

        logger.log_data(foot_pos, mtr_data, foot_goal, goal_mpos)

        send_mtr_current(bus, vctrl1.iqref, vctrl2.iqref)

        print("loop duration: {:.1f} ms\n".format(
            (get_time() - loop_start) * 1000))

    msg_handler = MessageHandler()
    msg_handler.set_id_handler(ArbitrationIds.status, mtr_data.set_status)
    msg_handler.set_id_handler(ArbitrationIds.current, mtr_data.set_current)
    msg_handler.set_id_handler(ArbitrationIds.position, on_position_msg)
    msg_handler.set_id_handler(ArbitrationIds.velocity, mtr_data.set_velocity)
    msg_handler.set_id_handler(ArbitrationIds.adc6, adc.set_values)

    # Don't process OptoForce messages. We don't need them here and the
    # computation is too expensive for the controller.
    #msg_handler.set_id_handler(ArbitrationIds.optoforce_trans,
    #        lambda msg: optoforce.receive_frame(msg.data))


    # wait for messages and update data
    for msg in bus:
        try:
            msg_handler.handle_msg(msg)
        except:
            print("\n\n=========== ERROR ============")
            print(traceback.format_exc())
            stop_system(bus)
            break
