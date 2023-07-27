import os
import numpy as np
from math import pi
from pyolp_robotics.robots import Robot

from OCC.Core.Graphic3d import (
    Graphic3d_NOM_STEEL,
)

class Kuka(Robot):
    def __init__(self, mesh=False):
        self.mesh = mesh
        super().__init__(mesh=self.mesh)
        self.available_models = ['Iiwa']
        self.file_format = ".klr"

    def post_processor(self, trajectory):
        txt = ''
        for i, pose in enumerate(trajectory):
            txt = txt + f'passe {i} \n'+ str(pose) + '\n'
        return txt

class Iiwa(Kuka):
    def __init__(self, mesh=False) -> None:
        self.mesh = mesh
        super().__init__(mesh=self.mesh)
        self.set_robot_model()
        self.process_directory()
        # self.initialise_robot()

    def set_robot_model(self):
        self.available_tools = ['tool_simple']
        dir_ = "resources/LBR_IIWA_14_R820"
        # get the dir full path
        dir_fullpath = os.path.join(os.path.dirname(__file__), dir_)
        self.path_directory = dir_fullpath
        self.robot_name = 'kuka_iiwa'
        self.nb_axes = 7
        self.color = 78
        self._reach = 600
        self.material = Graphic3d_NOM_STEEL
        # dh_params [n, 4]
        # |  d  |  a  |  alpha  |  theta  |
        # |  x  |  x  |    x    |    x    |
        # dh_params_14_R820
        self._dh_params = np.array([
            [  0.36, 0., pi/2, 0.],
            [  0., 0., -pi/2, 0.],
            [  0.42, 0., -pi/2, 0.],
            [  0., 0.,  pi/2, 0.],
            [  0.4, 0.,  pi/2, 0.],
            [  0., 0., -pi/2, 0.],
            [  0.126, 0., 0, 0.]])
        # radians
        self.home_config = [0, 0, 0, 0, 0, 0, 0]   
        self._joint_limits =  [
                        (-170 , 170),  # Limite articulation 0 (en degrés)
                        (-120 , 120),  # Limite articulation 1
                        (-170 , 170),  # Limite articulation 2
                        (-120 , 120),  # Limite articulation 3 
                        (-170 , 170),  # Limite articulation 4
                        (-120 , 120),  # Limite articulation 5
                        (-175 , 175)]  # Limite articulation 6
        
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