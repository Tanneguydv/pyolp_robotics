import pyolp_robotics.OCC_functions as occ
import numpy as np
import math
from math import pi
from ast import literal_eval as make_tuple
import os
from abc import ABC, abstractmethod

class Robot(ABC):
    def __init__(self, mesh=False):
        self.color = 0
        self.mesh = mesh
        self.has_tool = False

    @abstractmethod
    def post_processor(self, trajectory):
        pass

    @abstractmethod
    def set_robot_model(self):
        pass
           
    def load_geometry_from_step(self, path):
        step = occ.read_step_file(path)
        self.geometry = step

    def get_geometry(self):
        return self.geometry

    def process_directory(self):
        for filename in os.listdir(self.path_directory):
                f = os.path.join(self.path_directory, filename)
                if 'tools' in filename:
                    self.tools_directory = f
                if "_stp" in filename:
                    self.axes_step_directory = f
                if "axes_stl" in filename:
                    self.axes_stl_directory = f
                if "txt" in filename:
                    self.ax3_directory = f

    def get_ax3_from_DH(self, dh_params):
        # dh_params [n, 4]
        # |  d  |  a  |  alpha  |  theta  |
        # |  x  |  x  |    x    |    x    |
        previous_joint = occ.gp_Ax3()
        joints = []
        previous_alpha = 0
        previous_theta = 0
        previous_a_param = 0
        previous_d_param = 0
        for ax in dh_params:
            # alpha parameter
            joint_rota = occ.dir_rotation(
                previous_joint, previous_joint.Location(), 
                previous_joint.XDirection(), 
                previous_alpha)
            # theta parameter
            joint_rott = occ.dir_rotation(
                joint_rota, previous_joint.Location(), 
                previous_joint.Direction(), 
                previous_theta)
            # d parameter
            joint_d = occ.offset_with_dir_ax3(
                joint_rott, 
                previous_joint.Direction(), 
                previous_d_param*1000)
            # a parameter
            joint = occ.offset_with_dir_ax3(
                joint_d, 
                previous_joint.XDirection(), 
                previous_a_param*1000)
            joints.append(joint)
            previous_joint = joint
            previous_alpha = ax[2]
            previous_a_param = ax[1]
            previous_d_param = ax[0]
        return joints

    def initialise_robot(self, ax3s=None):
        if ax3s is None:    
            self.ax3s = self.get_ax3_from_DH(self.dh_params)    
            self.ax3s_original = self.get_ax3_from_DH(self.dh_params)
        else:
            self.ax3s = ax3s
            self.ax3s_original = ax3s
        
        self.axes = []
        self.axes_original = []
        for i in range(self.nb_axes):
            self.axes.append(0)
            self.axes_original.append(0)

        if self.mesh :
            files = os.listdir(self.axes_stl_directory)
            files = sorted(files, key=lambda x: int(os.path.splitext(x.split("_")[1])[0]))
            for filename in files:
                f = os.path.join(self.axes_stl_directory, filename)
                # checking if it is a file
                if os.path.isfile(f):
                    axe_number_str = f"{filename.split('_')[-1].split('.')[0]}"
                    # Convertir le axe_number en entier et l'ajouter à la liste des axes
                    axe_number = int(axe_number_str)
                    if axe_number == 0 :
                        self.base_shape = occ.read_stl_file(f)
                    else:
                        self.axes[axe_number-1] = occ.read_stl_file(f)
                        self.axes_original[axe_number-1] = occ.read_stl_file(f)
        else :
            files = os.listdir(self.axes_step_directory)
            files = sorted(files, key=lambda x: int(os.path.splitext(x.split("_")[1])[0]))
            for filename in files:
                f = os.path.join(self.axes_step_directory, filename)
                # checking if it is a file
                if os.path.isfile(f):
                    axe_number_str = f"{filename.split('_')[-1].split('.')[0]}"
                    # Convertir le axe_number en entier et l'ajouter à la liste des axes
                    axe_number = int(axe_number_str)
                    if axe_number == 0 :
                        self.base_shape = occ.read_step_file(f)
                    else:
                        self.axes[axe_number-1] = occ.read_step_file(f)
                        print("axe number -1 =", axe_number-1, filename)
                        self.axes_original[axe_number-1] = occ.read_step_file(f)

        self.config = []
        for a in self.home_config:
              self.config.append(0)
        print(" 0",self.config)
        self.change_config()
        print(" 1",self.config)
        self.initial_frame = self.ax3s[-1]

    def set_tool(self, tool_name):
        print('set ', tool_name)
        for filename in os.listdir(self.tools_directory):
            f = os.path.join(self.tools_directory, filename)
            # checking if it is a file
            if os.path.isfile(f):
                print('f=', f)
                if "txt" in filename and tool_name in filename:
                    with open(f, "r") as file:
                        for i in file.readlines():
                            try :
                                (xyz, nxyz, xdyz) = make_tuple(i)
                                ax3 = occ.gp_Ax3(occ.gp_Pnt(*xyz), occ.gp_Dir(*nxyz), occ.gp_Dir(*xdyz))
                            except ValueError:
                                (xyz, nxyz) = make_tuple(i)
                                ax3 = occ.gp_Ax3(occ.gp_Pnt(*xyz), occ.gp_Dir(*nxyz))
                            self.tool_ax3 = ax3
                            self.original_tool_ax3 = ax3
                            self.tool_flange_trsf = occ.get_trsf_2ax3(self.ax3s_original[-1], self.tool_ax3)
                            self.flange_tool_trsf = self.tool_flange_trsf.Inverted()
                            print('trsf =', self.flange_tool_trsf)
                elif tool_name in filename:
                    print('tool_name_for_step =', tool_name)
                    tool = occ.read_step_file(f)
                    new_flange = occ.create_compound([self.axes_original[-1], tool])
                    self.axes_original[-1] = new_flange
                    self.axes[-1] = new_flange
                    self.has_tool = True
                
    def get_ax3s(self):
        return self.ax3s
    
    def get_axes(self):
        return self.axes
    
    def get_ax3s_original(self):
        return self.ax3s_original
    
    def get_axes_original(self):
        return self.axes_original
    
    def get_home_config(self):
        return self.home_config
    
    def get_tcp_ax3(self):
        if self.has_tool:
            return self.tool_ax3
        else:
            return self.ax3s[-1]
    
    def get_compound(self):
        return occ.create_compound(self.axes)
    
    def go_home(self):
        self.config = self.home_config
        self.change_config()

    def forward_k(self, config):
        self.set_config(config)
        self.change_config()
        # ok_config = self.check_config_limits(config)
        # if ok_config:
        #     self.set_config(config)
        #     self.change_config()
        # else:
        #     print("config out of joint limits")

    def check_config_limits(self, config):
        for i, j in enumerate(config):
            j_degrees = math.degrees(j)
            if not self.joint_limits[i][0] < j_degrees < self.joint_limits[i][1]:
                return False
        return True

    def set_config(self, config):
        self.config = config

    def change_config(self): 
        self.set_original_axes()
        for i, angle in enumerate(self.config):
            for ind in range(i, self.nb_axes):
                # print("ind =", ind, "i = ", i, "angle =", angle)
                comp_rotated = occ.rotate_shape(self.axes[ind], occ.gp_Ax1(self.ax3s[i].Location(), self.ax3s[i].Direction()), angle, unite="rad")
                self.axes[ind] = comp_rotated
                ax3_rotated = self.ax3s[ind].Rotated(occ.gp_Ax1(self.ax3s[i].Location(), self.ax3s[i].Direction()), angle) 
                self.ax3s[ind] = ax3_rotated
        if self.has_tool:
            transformation_repere_tool = occ.gp_Trsf()
            transformation_repere_tool.SetTransformation(self.ax3s[-1], occ.gp_Ax3())
            transformation_repere_tool.Multiply(self.flange_tool_trsf)
            self.tool_ax3 = occ.gp_Ax3().Transformed(transformation_repere_tool)

    def set_original_axes(self): 
        self.axes = self.get_axes_original().copy()
        self.ax3s = self.get_ax3s_original().copy()


if __name__ == '__main__':
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

    rob = Robot()
    for v in rob.get_axes().values():
        display.DisplayShape(v)
    # display.DisplayShape(rob.get_axes_as_compound())
    display.FitAll()
    start_display()

