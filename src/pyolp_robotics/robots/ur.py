import os
import numpy as np
from math import pi
from pyolp_robotics.robots import Robot
from datetime import datetime

from OCC.Core.Graphic3d import (
    Graphic3d_NOM_STONE,
)

class UR(Robot):
    def __init__(self, mesh=False):
        self.mesh = mesh
        super().__init__(mesh=self.mesh)
        self.available_models = ['UR_10e']
        self.file_format = ".script"

    def post_processor(self, trajectory):
        # example of use, the URScript module from https://github.com/ErwinLutke/UR-Interface is useful
        acc = 0.1
        vit = 0.02
        rad = 0.001
        now = datetime.now()
        date_time = now.strftime("%y%m%d%H%M")
        txt = '# Program generated with Pyolp\n' + '#' + str(date_time) + '\n'
        for i, poses in enumerate(trajectory):
            txt = txt + "# passe number " + str(i) + "\n"
            for pose in poses:
                print(pose)
                rounded = [round(x, 6) for x in pose]
                rz, ry, rx, x, y, z = [i for i in rounded]
                pose = f"movej(p[{x}, {y}, {z}, {rx}, {ry}, {rz}], a={acc}, v={vit}, r={rad})"
                txt = txt + str(pose) + '\n'
            
        return txt

class UR_10e(UR):
    def __init__(self, mesh=False) -> None:
        self.mesh = mesh
        super().__init__(mesh=self.mesh)
        self.set_robot_model()
        self.process_directory()

    def set_robot_model(self):
        self.available_tools = ['grinding']
        dir_ = "resources/UR_10e"
        # get the dir full path
        dir_fullpath = os.path.join(os.path.dirname(__file__), dir_)
        self.path_directory = dir_fullpath
        self.robot_name = 'UR10_e'
        self._reach = 1300 # in millimeters
        self.nb_axes = 6
        self.color = 268
        self.material = Graphic3d_NOM_STONE
        # dh_params [n, 4]
        # |  d  |  a  |  alpha  |  theta  |
        # |  x  |  x  |    x    |    x    |
        # dh_params_UR https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
        self._dh_params = np.array([
            [  0.1273, 0., pi/2, 0.],
            [  0., -0.612, 0, 0.],
            [  0., -0.5723, 0, 0.],
            [  0.163941, 0.,  pi/2, 0.],
            [  0.1147, 0.,  -pi/2, 0.],
            [  0.0922, 0., 0, 0.]])   
        # radians
        self.home_config = [0, -pi/2, 0, -pi/2, 0, 0] 
        self._joint_limits =  [
                        (-360, 360),  
                        (-360, 360),  
                        (-360, 360), 
                        (-360, 360),   
                        (-360, 360), 
                        (-360, 360)] 
        
    @property
    def dh_params(self):
        return self._dh_params
    
    @property
    def reach(self):
        return self._reach
    
    @property
    def joint_limits(self):
        return self._joint_limits
    
    @joint_limits.setter
    def joint_limits(self, new_joint_limits):
        valid = True
        for j_limit in new_joint_limits:
            min_limit, max_limit = j_limit
            if not min_limit< 0 and not max_limit>0:
                valid = False
        if valid:
            self._joint_limits = new_joint_limits
        else: 
            print("please enter a valid range for joints limits")
    


        
