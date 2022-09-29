Example Scripts
===============

The `scripts` directory contains a bunch of scripts that can be used to test
the motor control board and serve as examples on how to use the CAN API
implemented in the `python_blmc` package.

Scripts that are prefixed with "leg_" are only meant for use with the hopper
leg prototype #1 (note that leg #2 is not supported).  The other scripts are
more general and can also be used for other applications.

Besides of that, there are two types of scripts. *Passive* ones and *active*
ones.  The passive ones only listen for messages but do not run the motors.
Therefore it is always save to call them.  Active scripts do enable and run the
motors.  Be careful when calling them.


General Scripts
---------------

### get_motor_info.py

*Passive*

Receive data from the board (status, motor current, position, etc.) and print
it in the terminal.

### read_optoforce.py

*Passive*

Receive data from the OptoForce sensor and print it in the terminal.

### measure_msg_freq.py

*Passive*

Measure the frequency with which messages are received.  Expects an arbitration
ID as argument (i.e. only analyses messags with the given ID). Example:

    measure_msg_freq.py 0x030    # measure frequency of position messages

### position_ctrl.py

*Active*

Basic example on how to run a position controller for the motors. The angular
position of the motors is given by the ADC inputs A6 and B6 (e.g. using
sliders).

Takes as arguments the max. position (in mechanical revolutions `[mrev]`) and
the gains of the PID controllers of both motors:

    position_ctrl.py  max_mrev  Kp1 Ki1 Kd1  Kp2 Ki2 Kd2

### velocity_ctrl.py

*Active*

Velocity controller for motor 1.  The velocity reference is given via a slider
that is connected to ADC A6.  The script takes as arguments the max. velocity
in krpm followed by the control gains of the PI controller:

    velocity_ctrl.py max_speed P I


Leg-specific Scripts
--------------------

The following scripts are only meant to be used with [leg prototype
#1](https://atlas.is.localnet/confluence/display/AMDW/Leg+Prototype+%231).


## leg_calibrate.py

*Passive*

**Run this before using any of the other scripts.**

Calibration procedure to determine the offsets that are used for position
initialization.  You are asked by the script to manually move the leg joints in
positive direction until the stop and press enter while holding them there.
Afterwards move the leg to the zero position (fully stretched) and again press
enter.

The offset between stop and zero is automatically stored to a file
"calibration_data.p" in the working directory.  This file is loaded by the
other scripts for the position initialization.

### leg_init_position.py

*Passive*

Demo on how to do the position initialization.  Like during calibration you are
asked to manually move the joints to the stop and press enter.

When done this script starts to print the current joint positions to the
terminal.  This way you can verify that the calibration was done correctly
(both joints should be at zero when the leg is stretched).

### leg_compute_kinematic.py

*Passive*

Demo on how to compute forward and inverse kinematics for the leg.  In each
iteration the script prints:

 * the angular position of the joints
 * Cartesian position of the foot based on forward kinematics
 * again the angular position of the joints based on inverse kinematics

This is useful to check if forward and inverse kinematics are computed
correctly.

Ambiguities in the inverse kinematics are solved by always using the solution
where the knee points leftwards.


### leg_move_sine.py

*Active*

Move both joints based on sine functions.  The foot should move up and down on
a vertical line while the knee moves on a horizontal line.

### leg_foot_pos_ctrl.py

*Active*

Control the position of the foot.  There are two modes: "slider" and "jump".

While the leg is moving, some data is logged and written to a file
"logged_data.txt".  Use the script leg_plot_logged_data.py to plot it.

#### Slider

When called with the argument "slider", the position of the foot can be
controlled via sliders.

The sliders have to be connected to the ADCs A6 and B6 of the board.  The
A-slider sets the vertical position of the foot, the B-slider the horizontal.
For the initial position (leg fully streched) set the A-slider to 0 and the
B-slider to half of its range.

#### Jump

When called with the argument "jump", the foot will move to a slightly crouched
position.  As soon as it touches the ground, the leg will slowly crouch down
and then immediately straighten the leg to jump.  This is repeated over and
over again.

In this mode touchdown detection is done by computing the force on the foot
based on motor torques.  Different gains for the PD controller are used
depending on if the leg touches the ground or not.


### leg_plot_logged_data.py

*Passive*

Plot data dumps from leg_foot_pos_ctrl.py.

The filename can be given as argument. Default is "logged_data.txt".
