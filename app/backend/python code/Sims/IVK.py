import ikpy.chain
import ikpy.utils.plot as plot_utils

import numpy as np
import time
import math

import ipywidgets as widgets
import serial

my_chain = ikpy.chain.Chain.from_urdf_file("ROBOT.urdf",active_links_mask=[True, True, True, True, True, True, True])

target_position = [ -0.5, 0, 0.50]

target_orientation = [-1, 0, 0]

ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")
print("The angles of each joints are : ", list(map(lambda r:math.degrees(r),ik.tolist())))