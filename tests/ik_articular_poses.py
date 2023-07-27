import os
import sys
sys.path.append(os.path.realpath(os.curdir)) 

import pyolp_robotics.OCC_functions as occ
from pyolp_robotics.robots.kuka import Iiwa
from pyolp_robotics.ikpy_planner import Planner 

from OCC.Display.SimpleGui import init_display
display, start_display, add_menu, add_function_to_menu = init_display()

rob = Iiwa()
rob.initialise_robot()
rob.set_tool('tool_simple')

coord_pt1 = occ.gp_Pnt(rob.reach/1.5, rob.reach/1.5, rob.reach/2)
dir_Z1 = occ.gp_Dir(0.5, 0, -1)
coord_pt2 = coord_pt1.Translated(occ.gp_Vec(300, -100, 0))
dir_Z2 = occ.gp_Dir(-0, 0, -1)

frames = [occ.gp_Ax3(coord_pt1, dir_Z1), occ.gp_Ax3(coord_pt2, dir_Z2)] 
    
for frame in frames:
    edge = occ.create_edge(frame.Location(), frame.Direction(), 150)
    display.DisplayShape(edge, color="blue")
    edge = occ.create_edge(frame.Location(), frame.XDirection(), 150)
    display.DisplayShape(edge, color="red")
    edge = occ.create_edge(frame.Location(), frame.YDirection(), 150)
    display.DisplayShape(edge, color="green")

planner = Planner(rob)
planner.set_frames([frames])
planner.compute_joints_angles()
poses = planner.get_joint_angle_trajectory()

configs = [poses[0]]
int_configs = planner.all_intermediate_poses(step=70, configs=poses, ax3s=frames)
configs.extend(int_configs)
configs.append(poses[1])
tcp_points = []
for i, config in enumerate(configs):
    rob.forward_k(config)
    display.DisplayShape(rob.get_compound(), color=rob.color, transparency=0.8, material=rob.material)
    tcp_points.append(rob.get_tcp_ax3().Location())
display.DisplayShape(occ.Bspline(tcp_points), color=23)
display.DisplayShape(rob.base_shape, color=rob.color, material=rob.material)

display.FitAll()
start_display() 