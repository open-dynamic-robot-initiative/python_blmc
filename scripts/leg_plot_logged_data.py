#!/usr/bin/env python
"""
Plot data dumps from leg_foot_pos_ctrl.py.

The filename can be given as argument. Default is "logged_data.txt".
"""
import sys
import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "logged_data.txt"

    data = np.loadtxt(filename)
    t = data[:, 0]
    foot_x = data[:, 1]
    foot_y = data[:, 2]
    foot_x_ref = data[:, 3]
    foot_y_ref = data[:, 4]
    mtr1_pos = data[:, 5]
    mtr1_pos_ref = data[:, 6]
    mtr2_pos = data[:, 7]
    mtr2_pos_ref = data[:, 8]

    # shift time to start at zero
    t = t - t[0]

    fig, sp = plt.subplots(2, 3)

    fig.suptitle(filename)

    sp[0, 0].set_title("foot x [m]")
    sp[0, 0].plot(t, foot_x)
    sp[0, 0].plot(t, foot_x_ref)

    sp[0, 1].set_title("foot y [m]")
    sp[0, 1].plot(t, foot_y)
    sp[0, 1].plot(t, foot_y_ref)

    sp[0, 2].set_title("foot (x,y)")
    sp[0, 2].plot(-foot_y, -foot_x)
    sp[0, 2].plot(-foot_y_ref, -foot_x_ref)

    sp[1, 0].set_title("Motor 1 [mrev]")
    sp[1, 0].plot(t, mtr1_pos)
    sp[1, 0].plot(t, mtr1_pos_ref)

    sp[1, 1].set_title("Motor 2 [mrev]")
    sp[1, 1].plot(t, mtr2_pos)
    sp[1, 1].plot(t, mtr2_pos_ref)

    plt.show()
