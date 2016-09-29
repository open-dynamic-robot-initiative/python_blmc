"""
Functions for the kinematics of leg number 1.
"""
import numpy as np


class LegProp:
    """Leg specific properties."""
    joint_1_gear_factor = 3
    joint_2_gear_factor = 3

    motor_1_kt = 0.018
    motor_2_kt = 0.018

    l1 = 0.1  # Length of link 1
    l2 = 0.1  # Length of link 2


def motor_positions_to_joint_angles(mpos1, mpos2):
    return (mpos1 * 2*np.pi / LegProp.joint_1_gear_factor,
            mpos2 * 2*np.pi / LegProp.joint_2_gear_factor)


def joint_angles_to_motor_position(th1, th2):
    return (th1 / (2*np.pi) * LegProp.joint_1_gear_factor,
            th2 / (2*np.pi) * LegProp.joint_2_gear_factor)


class Transformer:
    """Encapsulates all transformations that are based on the current leg
    configuraiton (i.e. forward kinematics).
    """

    def update_mtr_data(self, mtr_data):
        """Update internal motor data.

        This has to be done *before* any of the other methods is called!
        """
        self._mtr_data = mtr_data
        self._foot_pos = None
        (self.th1, self.th2) = motor_positions_to_joint_angles(
                mtr_data.mtr1.position.value,
                mtr_data.mtr2.position.value)

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

    def foot_position(self):
        """Compute foot position in base frame.

        Returns
        =======
        foot_pos : (float, float)
            (x,y)-position of the foot in the base frame.
        """
        if self._foot_pos is None:
            self._foot_pos = self.tf01().dot(self.tf12().dot(
                np.array([0, 0, 1])))[:2]
        return self._foot_pos

    def is_pose_safe(self):
        """Check if the given configuration is safe.

        Returns
        =======
        is_safe : bool
            True if the pose is safe, False if not.
        """
        return ((abs(self._mtr_data.mtr1.position.value) < 1.2)
                and (abs(self._mtr_data.mtr2.position.value) < 1.35)
                and (self._foot_pos[0] > -0.1)
                and (self._foot_pos[0] > 0 or abs(self._foot_pos[1]) > 0.04))

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


#def transform_optoforce_to_foot(force_s):
#    """Transfrom force vector from OptoForce frame to foot frame.
#
#    Parameter
#    =========
#    force_s : array
#        Force vector in sensor frame.
#
#    Returns
#    =======
#    force_f : array
#        Force vector in foot frame.
#    """
#    oost = 1. / np.sqrt(2)
#    R = np.array([[0, -oost,  oost],
#                  [0, -oost, -oost],
#                  [1,     0,     0]])
#    force_f = R.dot(force_s)
#    return force_f


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
    return joint_angles_to_motor_position(th1, th2)
