import os
import sys
sys.path.append(os.path.realpath(os.curdir)) 

import pyolp_robotics.OCC_functions as occ

from pyolp_robotics.robots.kuka import Iiwa
from pyolp_robotics.robots.ur import UR_10e
from pyolp_robotics.ikpy_planner import Planner

from OCC.Display.SimpleGui import init_display
display, start_display, add_menu, add_function_to_menu = init_display()


rob = UR_10e(mesh=False)
rob.initialise_robot()
rob.set_tool("grinding")

part_shape = occ.read_step_file('part_example.stp')
display.DisplayShape(part_shape)
edges = occ.TopExplorer(part_shape, "edge")
edge = edges[21]
waypoints = occ.DiscretizeWire(edge, 30)

planner = Planner(rob)

ax3s = []
for point in waypoints:
    ax3 = occ.gp_Ax3(point, occ.gp_Dir(0,0,-1))
    ax3s.append(ax3)

planner.set_frames([ax3s])
planner.compute_joints_angles()

joints = planner.get_joint_angle_trajectory()
print("self.config_list = ", joints)
for j in joints:
    rob.forward_k(j)
    display.DisplayShape(rob.get_compound(), color=rob.color, transparency=0.8, material=rob.material)
display.DisplayShape(rob.base_shape, color=rob.color, material=rob.material)
display.FitAll()
start_display()
