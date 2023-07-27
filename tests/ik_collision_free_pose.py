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

planner = Planner(rob)
config_initial = rob.config

ax3_1 = occ.gp_Ax3(occ.gp_Pnt(1112,-209,40), occ.gp_Dir(0,0,-1))

env_col_name ="env_col.stp"
env_col = occ.read_step_file(env_col_name)
planner.set_collision_environment(env_col)

display.DisplayShape(env_col, transparency=0.5)
display.DisplayShape(rob.base_shape, color=rob.color, material=rob.material)
display.DisplayShape(ax3_1.Location())

config_no_coll = planner.get_pose_collision_free(ax3_1, config_initial,  timeout=30, selfcollision_check=False)
rob.forward_k(config_no_coll)
if planner.success:
    display.DisplayShape(rob.get_compound(), color="green", material=rob.material)
else:
    display.DisplayShape(rob.get_compound(), color="red", material=rob.material)

display.FitAll()
start_display()


