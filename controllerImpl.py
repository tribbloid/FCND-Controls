import math

from dataclasses import dataclass
from pymavlink.quaternion import Quaternion
from pymavlink.rotmat import Matrix3

from controller import *


@dataclass
class PID(object):
    P: float = 0.2
    I: float = 0.0
    D: float = 0.0


np.set_printoptions(precision=3, suppress=True, floatmode="fixed") # disable scientific notation
MAX_THRUST_A = MAX_THRUST / DRONE_MASS_KG
MAX_THRUST_A_SQ = MAX_THRUST_A**2
@dataclass
class ControllerImpl(NonlinearController):
    position_xy: PID = PID(2, 0.001, 4)
    position_z: PID = PID(10, 0.5, 15)

    atti_xy: PID = PID(10)  # just P
    atti_z: PID = PID(2)  # just P

    torque_xy: PID = PID(20)  # just P
    torque_z: PID = PID(5)  # just P

    itrAtti: int = 0

    def __post_init__(self):
        self.MAX_THRUST_A = MAX_THRUST / DRONE_MASS_KG

        self.intg_dxy = np.array([0.0, 0.0])
        self.intg_dz = 0.0

    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                                 acceleration_ff=np.array([0.0, 0.0])):
        # return np.array([0,0])

        dxy = local_position_cmd - local_position
        self.intg_dxy += dxy
        ixy = self.intg_dxy
        d_dot_xy = local_velocity_cmd - local_velocity

        pid = self.position_xy
        axy = pid.P * dxy + pid.I * ixy + pid.D * d_dot_xy + acceleration_ff

        output = axy
        return output

    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude,
                         acceleration_ff=0.0):

        d_z = altitude_cmd - altitude
        self.intg_dz += d_z
        i_z = self.intg_dz
        d_dot_z = vertical_velocity_cmd - vertical_velocity

        pid = self.position_z
        a_up = pid.P * d_z + pid.I * i_z + pid.D * d_dot_z + acceleration_ff # already has gravity

        R = Matrix3()
        R.from_euler(*attitude)

        # [?, ?, up_dot_v]T = R [0,0,u1]T, get u1
        R33 = R.c.z

        # counter-projection
        output = a_up / R33
        clamped = min(MAX_THRUST, max(0, output))
        return clamped


    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame

        Args:
            acceleration_cmd: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton

        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        try:
            q_current = Quaternion(list(attitude))
            R_current = q_current.dcm
            R33 = R_current.c.z

            thrust_a = thrust_cmd / DRONE_MASS_KG
            thrust_a_z = thrust_a * R33

            a_cmd = np.array([*acceleration_cmd, - thrust_a_z])
            a_cmd_optimized = self.optimize_a_cmd(a_cmd, attitude, thrust_a)

            q_rect_body = q_current.inversed
            a_cmd_body = q_rect_body.transform(a_cmd_optimized)

            a_current_body = np.array([0, 0, -thrust_a])
            d_q = self.getRotationQuaternion(a_current_body, a_cmd_body)

            d_angle = d_q.euler

            # print(
            #     # str(q_current.euler), "\t==",
            #     q_target.euler, "\t=>", a_cmd)
            # a_cmd

        except AssertionError as ee:
            raise ee

        d_angle_xy = d_angle[[0, 1]]
        self.itrAtti += 1
        return d_angle_xy * self.atti_xy.P

    def optimize_a_cmd(self, a_cmd: np.array, attitude: np.array, thrust_a: float) -> np.array:
        """
        :param a_cmd: desired acceleration excluding gravity
        :return: a more realistic desired acceleration within its flight envelop:

        avoid flipping / downward thrust (this is to avoid gimbal lock, should be fixed later?)
        if max thrust is under powered, attempt to maintain z first

        theoretically should only use a_cmd, other args are just for show

        """
        info = [
            "itr_attitude:", self.itrAtti,
            "\tThrust-a:", thrust_a
        ]

        result = a_cmd.copy()
        if result[2] >= 0:
            result[2] = 0

        a_cmd_L2Sq = np.dot(result, result)

        if a_cmd_L2Sq > MAX_THRUST_A_SQ:
            info.extend([
                "\tunderpowered:", MAX_THRUST_A_SQ - a_cmd_L2Sq,
                "\tunable to achieve such acceleration! slowing down"
            ])
            max_xyL2Sq = MAX_THRUST_A_SQ - result[2]**2
            if max_xyL2Sq < 0:
                result[:] = [0, 0, - MAX_THRUST_A]
            else:
                xy_cmd = result[[0,1]]
                xy_L2Sq = np.dot(xy_cmd, xy_cmd)
                ratio = math.sqrt(max_xyL2Sq / xy_L2Sq)
                result[[0,1]] = result[[0,1]] * ratio

                # assert np.dot(result, result) == MAX_THRUST_A_SQ, str(np.dot(result, result)) + " != " + str(MAX_THRUST_A_SQ)


        print(*info)
        return result

    @staticmethod
    def getRotationQuaternion(v1: np.array, v2: np.array) -> Quaternion:
        """v1 and v2 are xyz vectors"""
        if np.dot(v1, v1) == 0 or np.dot(v2, v2) == 0 or np.array_equal(v1, v2):
            return Quaternion([1, 0, 0, 0])

        cross = np.cross(v1, v2)
        residual = math.sqrt(np.dot(v1, v1) * np.dot(v2, v2)) + np.dot(v1, v2)
        q_target = Quaternion(list([residual, *cross]))
        q_target.normalize()

        # try:
        #     q_target.dcm
        # except Exception as ee:
        #     raise ee

        # test
        # d_R = Matrix3()
        # d_R.from_euler(*q_target.euler)
        # fVec = Vector3(list(x))
        # fVec_target = Vector3(list(y))
        # fVec_rotated = d_R * fVec
        # assert (np.linalg.norm(fVec_target - fVec_rotated) <= 0.0001)

        return q_target

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate

        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians

        Returns: target yawrate in radians/sec
        """
        yaw_error = (yaw_cmd - yaw)
        if yaw_error > np.pi:
            yaw_error = yaw_error - 2.0 * np.pi
        elif yaw_error < -np.pi:
            yaw_error = yaw_error + 2.0 * np.pi

        d_yaw = yaw_error * self.atti_z.P
        return d_yaw

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        error_bodyRate = (body_rate_cmd - body_rate)
        d_bodyRate_xy = error_bodyRate[[0,1]] * self.torque_xy.P
        d_bdoyRate_z = error_bodyRate[2] * self.torque_z.P
        d_body_rate = np.array([*d_bodyRate_xy, d_bdoyRate_z])

        # jj = np.diag(MOI)
        # torque = d_body_rate + np.cross(body_rate, np.matmul(jj, body_rate))

        torque = MOI * d_body_rate

        torque_L2 = np.linalg.norm(torque)

        if torque_L2 >= MAX_TORQUE:
            torque =  (MAX_TORQUE / torque_L2) * torque
            assert(np.linalg.norm(torque) - 1 <= 0.00000001)

        return torque
