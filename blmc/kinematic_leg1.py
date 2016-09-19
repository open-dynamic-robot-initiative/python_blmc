"""
Functions for the kinematics of leg number 1.
"""
import numpy as np

def tf01(th1):
    """Get transformation matrix from base (0) to knee (1).

    Parameter
    =========
    th1 : float
        Angular position of joint 1 (hip) in radian.
    """
    Cth = np.cos(th1)
    Sth = np.sin(th1)
    return np.matrix([[Cth,  Sth, 0.1*Cth],
                      [Sth, -Cth, 0.1*Sth],
                      [  0,    0,       1]])


def tf12(th2):
    """Get transformation matrix from knee (1) to foot (2).

    Parameter
    =========
    th2 : float
        Angular position of joint 2 (knee) in radian.
    """
    Cth = np.cos(th2)
    Sth = np.sin(th2)
    return np.matrix([[Cth, -Sth, 0.1*Cth],
                      [Sth,  Cth, 0.1*Sth],
                      [  0,    0,       1]])


def inverse_kinematics(x, y):
    """Compute joint angles for given foot position.

    Ambiguties are solved by always choosing the configuration where the knee
    points in positive direction.

    Parameter
    =========
    x : float
        x-position of the foot in the base frame.
    y : float
        y-position of the foot in the base frame.

    Returns
    =======
    th1 : float
        Angluar position of joint 1 in radian
    th2 : float
        Angluar position of joint 2 in radian
    """
    l1 = 0.1
    l2 = 0.1
    atan2 = np.arctan2

    # done with much help of
    # http://www.diag.uniroma1.it/~deluca/rob1_en/10_InverseKinematics.pdf

    c2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    s2 = np.sqrt(1 - c2**2)

    th2 = atan2(s2, c2)
    th1 = atan2(y, x) + atan2(l2*s2, l1+l2*c2)

    return (th1, th2)


def mrev_to_rad(mrev):
    """Convert mrev to radian."""
    return mrev * 2*np.pi


def is_pose_safe(mpos1, mpos2, foot_0):
    """Check if the given configuration is safe.

    Parameter
    =========
    mpos1 : float
        Angular position of motor 1 in mrev.
    mpos2 : float
        Angular position of motor 2 in mrev.
    foot_0 : (float, float)
        Position of the foot in base frame.

    Returns
    =======
    is_safe : bool
        True if the pose is safe, False if not.
    """
    return ((abs(mpos1) < 1.1) and (abs(mpos2) < 1.1)
        and (foot_0[0] > -0.1) and (foot_0[0] > 0 or abs(foot_0[1]) > 0.04))



