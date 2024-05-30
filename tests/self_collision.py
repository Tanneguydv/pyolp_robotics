import os
import sys
from OCC.Core.AIS import AIS_Shape

from pyolp_robotics.collision import check_two_part_collisions, Ais_Collision

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
for nr_jnt, axe in enumerate(rob.get_axes()[:-1]):
    collision, collider = check_two_part_collisions(axe, axe_with_tool, display_ais=display, get_collider=True)
    if collision:
        faces_a, faces_b = collider.get_collision_sets()
        print(f"joint: {nr_jnt} interferes with tool")
        display.DisplayShape(faces_a, color="red")
        display.DisplayShape(faces_b, color="green")
        print(f" len collision sets: {len(faces_a)}, {len(faces_b)}")
        print("if of similar length, that's suspicious")

display.FitAll()
start_display() 