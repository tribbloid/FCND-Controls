# -*- coding: utf-8 -*-
"""
Starter code for the controls project.
This is the solution of the backyard flyer script, 
modified for all the changes required to get it working for controls.
"""

import time
from enum import Enum

import numpy as np
from udacidrone.connection import MavlinkConnection  # noqa: F401
from udacidrone.messaging import MsgID

from controllerImpl import ControllerImpl
from controllerReference import ControllerReference
from controls_flyer import ControlsFlyer
from unity_drone import UnityDrone

if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://127.0.0.1:5760')
    drone = ControlsFlyer(conn, ControllerReference())
    time.sleep(2)
    drone.start()
    drone.print_mission_score()
