"""
Functions for the kinematics of leg number 1.
"""
import numpy as np


class LegProp:
    joint_1_gear_factor = 3
    joint_2_gear_factor = 3

    l1 = 0.1  # Length of link 1
    l2 = 0.1  # Length of link 2

    motor_1_kt = 0.018
    motor_2_kt = 0.018


def _motor_positions_to_joint_angles(mpos1, mpos2):
    return (mpos1 * 2*np.pi / LegProp.joint_1_gear_factor,
            mpos2 * 2*np.pi / LegProp.joint_2_gear_factor)


class Transformer:

#    def __init__(self):

    def update_mtr_data(self, mtr_data):
        (self.th1, self.th2) = _motor_positions_to_joint_angles(
                mtr_data.mtr1.position.value,
                mtr_data.mtr2.position.value)

    def foot_position(self):
        """Compute foot position in base frame.

        Returns
        =======
        foot_0 : (float, float)
            (x,y)-position of the foot in the base frame.
        """
        foot_2 = np.array([0, 0, 1])
        foot_0 = self.tf01().dot(self.tf12().dot(foot_2))
        return foot_0[:2]

    def tf01(self):
        """Get transformation matrix from base (0) to knee (1).

        Parameter
        =========
        th1 : float
            Angular position of joint 1 (hip) in radian.
        """
        Cth = np.cos(self.th1)
        Sth = np.sin(self.th1)
        return np.array([[Cth,  Sth, LegProp.l1 * Cth],
                         [Sth, -Cth, LegProp.l1 * Sth],
                         [  0,    0,                1]])

    def tf12(self):
        """Get transformation matrix from knee (1) to foot (2).

        Parameter
        =========
        th2 : float
            Angular position of joint 2 (knee) in radian.
        """
        Cth = np.cos(self.th2)
        Sth = np.sin(self.th2)
        return np.array([[Cth, -Sth, LegProp.l2 * Cth],
                         [Sth,  Cth, LegProp.l2 * Sth],
                         [  0,    0,                1]])

    def transform_optoforce_to_base(self, force_s):
        """Transfrom force vector from OptoForce frame to foot frame.

        Parameter
        =========
        force_s : array
            Force vector in sensor frame.

        Returns
        =======
        force_f : array
            Force vector in foot frame.
        """
        angle = self.th1 - self.th2 + np.pi/4
        c = np.cos(angle)
        s = np.sin(angle)
        R = np.array([[1,  0, 0],
                      [0,  c, s],
                      [0, -s, c]])
        force_b = R.dot(force_s)
        # reorder axes
        force_b = np.array([force_b[2], force_b[1], -force_b[0]])
        return force_b

    def foot_force(self, iq1, iq2):
        """Compute the force applied to the foot based on motor torques.

        Parameter
        ========
        iq1 : float
            Current I_q of motor 1 in A.
        iq2 : float
            Current I_q of motor 2 in A.

        Returns
        =======
        force : array([fx, fy])
            Force applied to the foot divided into x- and y-components.
        """
        l1 = LegProp.l1
        l2 = LegProp.l2
        q1 = self.th1
        q2 = self.th2
        t1 = iq1 * LegProp.motor_1_kt * LegProp.joint_1_gear_factor
        t2 = iq2 * LegProp.motor_2_kt * LegProp.joint_2_gear_factor
        sin = np.sin
        cos = np.cos

        # equations generated and simplified with SymPy
        fx = -(l1*l2*t1*sin(q2) + (l1*cos(q1) + l2*cos(q1 - q2))*(l2*t1*sin(q1 -
            q2) + t2*(l1*sin(q1) + l2*sin(q1 - q2))))/(l1*l2*(l1*sin(q1) +
                l2*sin(q1 - q2))*sin(q2))
        fy = -(l1*t2*sin(q1) + l2*t1*sin(q1 - q2) + l2*t2*sin(q1 -
            q2))/(l1*l2*sin(q2))

        return np.array((fx, fy))







def tf01(th1):
    """Get transformation matrix from base (0) to knee (1).

    Parameter
    =========
    th1 : float
        Angular position of joint 1 (hip) in radian.
    """
    Cth = np.cos(th1)
    Sth = np.sin(th1)
    return np.matrix([[Cth,  Sth, LegProp.l1 * Cth],
                      [Sth, -Cth, LegProp.l1 * Sth],
                      [  0,    0,                1]])


def tf12(th2):
    """Get transformation matrix from knee (1) to foot (2).

    Parameter
    =========
    th2 : float
        Angular position of joint 2 (knee) in radian.
    """
    Cth = np.cos(th2)
    Sth = np.sin(th2)
    return np.matrix([[Cth, -Sth, LegProp.l2 * Cth],
                      [Sth,  Cth, LegProp.l2 * Sth],
                      [  0,    0,                1]])


def transform_optoforce_to_foot(force_s):
    """Transfrom force vector from OptoForce frame to foot frame.

    Parameter
    =========
    force_s : array
        Force vector in sensor frame.

    Returns
    =======
    force_f : array
        Force vector in foot frame.
    """
    oost = 1. / np.sqrt(2)
    R = np.array([[0, -oost,  oost],
                  [0, -oost, -oost],
                  [1,     0,     0]])
    force_f = R.dot(force_s)
    return force_f


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
    l1 = LegProp.l1
    l2 = LegProp.l2
    atan2 = np.arctan2

    # done with much help of
    # http://www.diag.uniroma1.it/~deluca/rob1_en/10_InverseKinematics.pdf

    c2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    c2 = min(c2, 1.0) # avoid exposions due to numerical instability
    s2 = np.sqrt(1 - c2**2)

    th2 = atan2(s2, c2)
    th1 = atan2(y, x) + atan2(l2*s2, l1+l2*c2)

    return (th1, th2)


def inverse_kinematics_mrev(x, y):
    (th1, th2) = inverse_kinematics(x, y)
    return (rad_to_mrev(th1) * LegProp.joint_1_gear_factor,
            rad_to_mrev(th2) * LegProp.joint_2_gear_factor)


def mrev_to_rad(mrev):
    """Convert mrev to radian."""
    return mrev * 2*np.pi


def rad_to_mrev(rad):
    """Convert radian to mrev."""
    return rad / (2*np.pi)


def foot_position(mpos1, mpos2):
    """Compute foot position in base frame.

    Parameter
    =========
    mpos1 : float
        Angular position of motor 1 in mrev.
    mpos2 : float
        Angular position of motor 2 in mrev.

    Returns
    =======
    foot_0 : (float, float)
        (x,y)-position of the foot in the base frame.
    """
    th1 = mrev_to_rad(mpos1) / LegProp.joint_1_gear_factor
    th2 = mrev_to_rad(mpos2) / LegProp.joint_2_gear_factor
    foot_2 = np.matrix([0, 0, 1]).transpose()
    foot_0 = tf01(th1) * tf12(th2) * foot_2
    foot_0 = np.asarray(foot_0.transpose())[0, :2]

    return foot_0


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
    return ((abs(mpos1) < 1.2) and (abs(mpos2) < 1.35)
        and (foot_0[0] > -0.1) and (foot_0[0] > 0 or abs(foot_0[1]) > 0.04))
