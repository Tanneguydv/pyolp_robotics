import os
import sys
sys.path.append(os.path.realpath(os.curdir)) 

import pyolp_robotics.OCC_functions as occ
from pyolp_robotics.robots.ur import UR_10e

from OCC.Display.SimpleGui import init_display
display, start_display, add_menu, add_function_to_menu = init_display()

rob = UR_10e(mesh=False)
rob.initialise_robot()
rob.set_tool("grinding")

config_collision =  [-2.13173836, -1.29331347,  0.7562257,   0.53708777, -2.13173836,  0.]

rob.forward_k(config_collision)
display.DisplayShape(rob.get_compound(), transparency=0.8)

collision, colliding_parts = occ.fcl_collisions_collection_shapes(rob.get_axes(), stop_at_first=False)
for shape in colliding_parts:
    display.DisplayShape(shape, color="red")
    
display.FitAll()
start_display() 