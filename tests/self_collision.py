import os
import sys
sys.path.append(os.path.realpath(os.curdir)) 

import pyolp_robotics.OCC_functions as occ
from pyolp_robotics.robots.ur import UR10

from OCC.Display.SimpleGui import init_display
display, start_display, add_menu, add_function_to_menu = init_display()

rob = UR10(mesh=False)
rob.initialise_robot()
rob.set_tool("grinding")

config_collision =  [-2.13173836, -1.29331347,  0.7562257,   0.53708777, -2.13173836,  0.]

rob.forward_k(config_collision)
display.DisplayShape(rob.get_compound(), transparency=0.8)

axe_with_tool =  rob.get_axes()[-1]
for axe in rob.get_axes()[:-1]:
    collision = occ.check_two_part_collisions(axe, axe_with_tool)
    if collision:
        display.DisplayShape(axe, color="red")
        display.DisplayShape(axe_with_tool, color="red")
    
display.FitAll()
start_display() 