import os
import sys
sys.path.append(os.path.realpath(os.curdir)) 

import pyolp_robotics.OCC_functions as occ
from pyolp_robotics.robots.ur import UR_10e
from pyolp_robotics.ikpy_planner import Planner

from OCC.Display.SimpleGui import init_display
display, start_display, add_menu, add_function_to_menu = init_display()

rob = UR_10e(mesh=False)
rob.initialise_robot()
rob.set_tool("grinding")
# set new joint limits 
rob.joint_limits = [
                        (-360, 360), 
                        (-180, 180), 
                        (-180, 180),  
                        (-360, 360), 
                        (-360, 360), 
                        (-360, 360)]
part_shape = occ.Cylinder(occ.gp_Ax2(), 200, 200)
part_shape = occ.translate_shp(part_shape, occ.gp_Vec(rob.reach/2, rob.reach/3, 0))
display.DisplayShape(part_shape, transparency=0.8)
sections = occ.slice_shape(part_shape)
edges = occ.TopExplorer(sections, "edge")

for edge in edges[:-1]:
    pipe = occ.Pipe(edge, 2.5)
    display.DisplayShape(pipe, color=2) 
display.DisplayShape(edges[-1])

waypoints = occ.DiscretizeWire(edges[-1], 30)
ax3s = []
for point in waypoints:
    ax3 = occ.gp_Ax3(point, occ.gp_Dir(0,0,-1))
    ax3s.append(ax3)


planner = Planner(rob)
planner.set_frames([ax3s])
planner.compute_joints_angles()

joints = planner.get_joint_angle_trajectory()
print("self.config_list = ", joints)
total_nb_poses = len(joints)
for i, j in enumerate(joints):
    rob.forward_k(j)
    display.DisplayShape(rob.get_compound(), color=rob.color, transparency=(i/total_nb_poses), material=rob.material)
display.DisplayShape(rob.base_shape, color=rob.color, material=rob.material)
display.FitAll()
start_display()
