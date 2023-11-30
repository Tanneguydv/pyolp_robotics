from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
import math
import numpy as np
import random
import time
from heapq import heappop, heappush
import heapq
from math import pi, sqrt, sin, cos, atan2, radians, degrees

import pyolp_robotics.OCC_functions as occ

class Planner(object):
    def __init__(self, robot) -> None:
        self.robot = robot
        self.chain = self.create_sym_robot()
        self.inter_values = []
        self.joint_angle_trajectory = []
        self.initial_position = None
        self.collision_environment = occ.Sphere(occ.gp_Pnt(), 1)
        self.tolerance = 0.1

    def create_sym_robot(self):
        # get flange offset value
        self.flange_offset = abs(self.robot.dh_params[-1][0])*1000 
        # create a list of links for the robot
        links = []
        self.link_lengths = []
        for i, dh in enumerate(self.robot.dh_params):
            link_length = abs(dh[1]) # use absolute value of a as link length
            self.link_lengths.append(link_length)
            link = DHLink(d=dh[0], a=dh[1], alpha=dh[2], theta=dh[3], length=link_length)
            link.bounds = math.radians(self.robot.joint_limits[i][0]), math.radians(self.robot.joint_limits[i][1])
            links.append(link)
        # Create a chain with the links
        chain = Chain(links, name=self.robot.robot_name)
        return chain

    def set_frames(self, ax3s):
        self.frames = []
        self.waypoints = []
        if self.robot.has_tool:
            self.process_ax3_to_frame_tool(ax3s)
        else:
            self.process_ax3_to_frame(ax3s)

    def set_initial_position(self, config):
        self.initial_position = config

    def set_collision_environment(self, env_col):
        self.collision_environment = env_col
    
    def process_ax3_to_frame_tool(self, ax3s):
        ax1 = occ.gp_Ax3(occ.gp_Pnt(), occ.gp_Dir(0,0,-1), occ.gp_Dir(1,0,0))
        for i, frames in enumerate(ax3s):
            self.waypoints.append(i)
            self.waypoints[i] = []
            for frame in frames:
                # apply transformations to the frame
                frame_trsf= occ.gp_Trsf()
                frame_trsf.SetTransformation(frame, occ.gp_Ax3())
                frame_trsf.Multiply(self.robot.tool_flange_trsf)
                # define the coordinates of the transformed frame
                ax3 = occ.gp_Ax3().Transformed(frame_trsf)
                ax3 = occ.offset_with_dir_ax3(ax3, ax3.Direction(), self.flange_offset)
                self.ax3_recomputed = ax3
                euler_angle = occ.get_euler_angles(ax3, ax1, degres=False)
                self.waypoints[i].append(
                    [euler_angle[2], 
                    euler_angle[1], 
                    euler_angle[0],
                    ax3.Location().X()/1000, 
                    ax3.Location().Y()/1000, 
                    ax3.Location().Z()/1000])
                self.frames.append([
                    [
                        ax3.Location().X()/1000,
                        ax3.Location().Y()/1000,
                        ax3.Location().Z()/1000,
                    ],
                    [
                        ax3.Direction().X(),
                        ax3.Direction().Y(),
                        ax3.Direction().Z(),
                    ]])
    
    def process_ax3_to_frame(self,ax3s):
        ax1 = occ.gp_Ax3(occ.gp_Pnt(), occ.gp_Dir(0,0,1), occ.gp_Dir(1,0,0))
        for i, ax3_ in enumerate(ax3s):
            self.waypoints.append(i)
            self.waypoints[i] = []
            for ax3 in ax3_:
                euler_angle = occ.get_euler_angles(ax3, ax1, degres=False)
                self.waypoints[i].append(
                    [euler_angle[2], 
                    euler_angle[1], 
                    euler_angle[0],
                    ax3.Location().X()/1000, 
                    ax3.Location().Y()/1000, 
                    ax3.Location().Z()/1000])
                self.frames.append([
                    [
                        ax3.Location().X()/1000,
                        ax3.Location().Y()/1000,
                        ax3.Location().Z()/1000,
                    ],
                    [
                        ax3.Direction().X(),
                        ax3.Direction().Y(),
                        ax3.Direction().Z(),
                    ]])
    
    def get_frames(self):
        return self.frames
    
    def get_waypoints(self):
        return self.waypoints
    
    def get_nb_waypoints(self):
        flat_list = [item for sublist in self.waypoints for item in sublist]
        return len(flat_list)

    def compute_joints_angles(self, seed_initial=True):
        self.joint_angle_trajectory = []
        for frame in self.frames:
            target_position, target_orientation = frame
            joint_angles = self.chain.inverse_kinematics(target_position=target_position, target_orientation=target_orientation, orientation_mode="Z",
                                        initial_position=self.initial_position)
            self.joint_angle_trajectory.append(joint_angles)
            if seed_initial:
                self.initial_position = joint_angles
    
    def get_pose_collision_free(self, ax3, config_initial, timeout=100, selfcollision_check=False):
        self.initial_position = config_initial
        self.set_frames([[ax3]])
        start_time = time.time()
        recompute = True
        self.success = True
        while recompute :
            while True :
                try:
                    self.compute_joints_angles(seed_initial=True)
                except ValueError: # deal with infeasible least square methods
                    print("error, retrying ...")
                    self.initial_position = self.get_random_config()
                    continue
                break
            config_ikpy = self.get_joint_angle_trajectory()
            print("config_ikpy = ", config_ikpy[-1])
            self.robot.set_config(config_ikpy[-1])
            self.robot.change_config()
            if selfcollision_check :
                all_shapes = self.robot.get_axes()
                all_shapes.append(self.collision_environment)
            else :
                all_shapes = [self.collision_environment, self.robot.get_compound()]
            collision_non_ang_checked, shapes_colliding = occ.check_collections_collisions(all_shapes)
            if not collision_non_ang_checked:
                # config = self.rebase_config_domain(config_ikpy[-1])
                articular_limit_ok = self.check_articular_limits(config_ikpy[-1])
                collision_nondist_checked = articular_limit_ok
                if not collision_nondist_checked : #TODO works only with TCP
                    final_distance = self.robot.get_tcp_ax3().Location().Distance(ax3.Location())
                    print("final distance = ", final_distance)
                    if final_distance < self.tolerance:
                        recompute = False
            print("recomputing? ", recompute)
            config_initial = self.get_random_config()
            self.initial_position = config_initial
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout:
                recompute = False
                self.success = False

        config = config_ikpy[-1]         
        end_time = time.time()
        # Elapsed time
        elapsed_time = end_time - start_time
        # Display of elapsed time
        print("Elapsed time : {:.2f} seconds".format(elapsed_time))  
        print(f"Calculation success is {self.success}") 
        return config
    
    def get_random_config(self):
        random_config = []
        for j_limits in self.robot.joint_limits:
            random_config.append(radians(random.randint(*j_limits)))
        return random_config
    
    def rebase_config_domain(self, config):
        results = []
        for a in config:
            radians = a
            result = math.fmod(radians, 2*math.pi)
            if result > math.pi:
                result -= 2*math.pi
            elif result < -math.pi:
                result += 2*math.pi
            results.append(result)
        return results      

    def check_articular_limits(self, config):
        for i, val in enumerate(config):
            min_limit, max_limit = math.radians(self.robot.joint_limits[i][0]), math.radians(self.robot.joint_limits[i][1])
            # print("articular limit = " , min_limit, val, max_limit)
            if val < min_limit or val > max_limit:
                return True
        return False
    
    def intermediate_pose(self, nb_pt, config_1, step_change_config): 
        intermediate_config = []
        for i in range(nb_pt):
            config_1 = [x + step for x, step in zip(config_1, step_change_config)]
            intermediate_config.append(config_1)        
        return intermediate_config
    
    def all_intermediate_poses(self, step, configs, ax3s): 
        nb_config = []
        for i in range(len(ax3s)-1):
            print(str(i) + " / " + str(len(ax3s)))
            dist = occ.distance_two_points( ax3s[i+1].Location(), ax3s[i].Location())
            nb_pt = int(dist/step)
            print("nb_pt = ", nb_pt)
            nb_config.append(nb_pt)

        step_change_config = []
        for i, (config, nb) in enumerate(zip(configs, nb_config), start=1):
            change = configs[i] - configs[i-1]
            step = change / nb
            print("step = ", step)
            step_change_config.append(step)

        intermediate_config = []
        for  i, (config, nb_pt, step_change_conf) in enumerate(zip(configs, nb_config, step_change_config)):
            int_configs = self.intermediate_pose(nb_pt, config, step_change_conf)
            intermediate_config.append(int_configs)
        flat_list = [item for sublist in intermediate_config for item in sublist]
        return flat_list
    
    def compute_ik_frames_spline(self, spline, ax3_to, step=100, reverse=False, config_initial=None, with_collision=False):
        curve_length = occ.curve_length(spline) 
        nb_pt = int(curve_length / step)
        if config_initial is not None:
            self.robot.set_config(config_initial)
            self.robot.change_config()
        pnts = occ.DiscretizeWire(spline, nb_pt)
        if reverse:
            pnts.reverse()
        ax3s = []
        for p in pnts:
            ax3 = occ.gp_Ax3(p, ax3_to.Direction())
            ax3s.append(ax3)
        if not with_collision :
            self.set_frames([ax3s])
            self.compute_joints_angles(seed_initial=True)
            config_ikpy = self.get_joint_angle_trajectory()
        else :
            config_ikpy = []
            config_no_collision = config_initial
            for ax3 in ax3s:
                try :
                    config_no_collision = self.get_pose_collision_free(ax3, config_initial=config_no_collision, timeout=50)
                    config_ikpy.append(config_no_collision)
                except TypeError:
                    break
        ax3s.reverse()
        config_ikpy.reverse()
        return config_ikpy, ax3s

    def get_joint_angle_trajectory(self):
        return self.joint_angle_trajectory

    def get_axis_values(self):
        return self.inter_values
    
    def get_all_axis_values(self):
        flat_list_axis_values = [item for sublist in self.inter_values for item in sublist]
        return flat_list_axis_values
    
    def set_tolerance(self, tolerance=0.1):
        self.tolerance = tolerance

