import math

from pymavlink.quaternion import Quaternion

import numpy as np

from controllerImpl import PID, ControllerImpl

pid = PID(1/2, 0, 1/2)

cc = ControllerImpl(
    pid, pid, pid
)

def test_quaternion_spike():

    q = Quaternion([1,0,0,0])
    e = q.euler
    assert(list(e) == [0,0,0])

    q2 = Quaternion([-math.pi/2, 0, 0])
    q3 = Quaternion([0, -math.pi/2, 0])

    assert(q2 != q3)




def test_body_rate():

    result = cc.body_rate_control(np.array([2,2,2]), np.array([0.5,0.5,0.5]))
    assert(result.shape == (3,))
    print(result)


def test_getRotationQuaternion():
    #  rolling
    v1 = np.array([0,1,0])
    v2 = np.array([0,0,1])

    q: Quaternion = cc.getRotationQuaternion(v1,v2)
    nautical = q.euler
    assert(np.linalg.norm(nautical - np.array([math.pi/2, 0, 0])) < 0.00000001)
    cross = np.cross(v1, v2)

    cos = np.dot(v1, v2)
    angle = math.acos(cos)

    nautical2 = cross * (angle / math.sin(angle))
    # cross * (theta / sin theta)
    # we have cos theta
    # shortcut?

    norm = np.linalg.norm(cross)
    angle = math.asin(norm)
    ratio = angle / norm
    nautical2 = cross * ratio

    assert(nautical2 == nautical2)


def test_roll_pitch():

    # half right => leveled
    result = cc.roll_pitch_controller(np.array([0,0]), np.array([math.pi/4,0,0]), 4)
    assert(np.linalg.norm(result - [- math.pi/8, 0]) <= 0.0000001)

    # full right => leveled
    result = cc.roll_pitch_controller(np.array([0,0]), np.array([math.pi/2,0,0]), 4)
    assert(np.linalg.norm(result - [- math.pi/4, 0]) <= 0.0000001)

    # right flipped => leveled
    result = cc.roll_pitch_controller(np.array([0,0]), np.array([math.pi,0,0]), 4)
    assert(np.linalg.norm(result - [- math.pi/2, 0]) <= 0.0000001)

    # half forward => leveled
    result = cc.roll_pitch_controller(np.array([0,0]), np.array([0,math.pi/4,0]), 4)
    assert(np.linalg.norm(result - [0, - math.pi/8]) <= 0.0000001)

    # cannot recover from full forward, will cause gimbal lock!

    # cannot recover from forward flipped
    # result = cc.roll_pitch_controller(np.array([0,0]), np.array([0,2 * math.pi/3,0]), 4)
    # assert(np.linalg.norm(result - [0, - math.pi/3]) <= 0.0000001)
