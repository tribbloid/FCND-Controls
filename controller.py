"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import math

import numpy as np
from dataclasses import dataclass
from pymavlink.quaternion import Quaternion
from pymavlink.rotmat import Matrix3

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0


class NonlinearController(object):

    def __init__(self):
        """Initialize the controller object and control gains"""
        return

    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory

        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds

        Returns: tuple (commanded position, commanded velocity, commanded yaw)

        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]


        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]

            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]

        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]

                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]

        position_cmd = (position1 - position0) * \
                       (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)
        
        
        return position_cmd, velocity_cmd, yaw_cmd
    
    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                                 acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_ff: feedforward acceleration command
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        return np.array([0.0, 0.0])

    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)

        Returns: thrust command for the vehicle (+up)

        Hints:
         - Note that the return variable is thrust, not just acceleration, so make sure to take into account the
        drone's mass accordingly!
        """
        return 0.0
        

    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame

        Args:
            acceleration_cmd: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton

        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s

        Hints:
         - Note that the last command parameter is thrust, not acceleration.  If you plan on using acceleration in your
         controller, make sure to account for the drone's mass!

        """
        return np.array([0.0, 0.0])

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters

        Hints:
         - Note the output of this function in the desired moment for the drone, so make sure to take the moment of
         inertia of the drone into account accordingly!
        """
        return np.array([0.0, 0.0, 0.0])

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate

        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians

        Returns: target yawrate in radians/sec
        """
        return 0.0


@dataclass
class PID(object):
    P: float = 0.2
    I: float = 0.0
    D: float = 0.0


@dataclass
class ControllerImpl(NonlinearController):
    torque: PID = PID(0.2)  # just P
    atti: PID = PID(0.5)  # just P
    position: PID = PID(0.5, 0, 0.1)

    def __post_init__(self):
        self.intg_xy = np.array([0, 0])
        self.intg_z = 0

    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                                 acceleration_ff=np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_ff: feedforward acceleration command

        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        return np.array([0,0])

        # d_xy = local_position_cmd - local_position
        # self.intg_xy += d_xy
        # i_xy = self.intg_xy
        # d_dot_xy = local_velocity_cmd - local_velocity
        #
        # pid = self.position
        # dot_v = pid.P * d_xy + pid.I * i_xy + pid.D * d_dot_xy + acceleration_ff
        #
        # output = dot_v
        # return output


    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude,
                         acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)

        Returns: thrust command for the vehicle (+up)
        """

        d_z = altitude_cmd - altitude
        self.intg_z += d_z
        i_z = self.intg_z
        d_dot_z = vertical_velocity_cmd - vertical_velocity

        pid = self.position
        up_dot_v = pid.P * d_z + pid.I * i_z + pid.D * d_dot_z + acceleration_ff

        R = Matrix3()
        R.from_euler(*attitude)

        # [?, ?, up_dot_v]T = R [0,0,u1]T, get u1
        R33 = R.c.z

        # counter-projection
        output = up_dot_v / R33
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
        q_current = Quaternion(list(attitude))  # xyz

        ma_xy = DRONE_MASS_KG * acceleration_cmd
        ma_z = - math.sqrt(thrust_cmd**2 - np.inner(ma_xy, ma_xy))
        ma = np.array([*ma_xy, ma_z])

        # test
        # ma_norm = np.linalg.norm(ma)
        # assert(ma_norm == thrust_cmd)

        f_body = np.array([0, 0, -thrust_cmd])
        q_target = self.getRotationQuaternion(f_body, ma)

        d_q = q_target * q_current.inversed
        d_q.normalize()
        try:
            d_angle = d_q.euler
        except AssertionError as ee:
            raise ee

        d_angle_xy = d_angle[[0, 1]]
        return d_angle_xy * self.atti.P

    @staticmethod
    def getRotationQuaternion(v1: np.array, v2: np.array) -> Quaternion:
        """v1 and v2 are xyz vectors"""
        if np.array_equal(v1, v2):
            return Quaternion([1, 0, 0, 0])

        cross = np.cross(v1, v2)
        residual = math.sqrt(np.dot(v1, v1) * np.dot(v2, v2)) + np.dot(v1, v2)
        q_target = Quaternion(list([residual, *cross]))
        q_target.normalize()

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
        d_yaw = (yaw_cmd - yaw) * self.atti.P
        return d_yaw

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        d_body_rate = (body_rate_cmd - body_rate) * self.torque.P
        jj = np.diag(MOI)
        torque = d_body_rate + np.cross(body_rate, np.matmul(jj, body_rate))

        return torque
