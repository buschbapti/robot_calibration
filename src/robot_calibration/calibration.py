#!/usr/bin/env python
import rospy
import tf
import json
import rospkg
import yaml
import numpy as np
from os import system
from scipy.optimize import minimize
import time
from os.path import exists
from os.path import join
from os import makedirs
from robot_calibration import transformations


class Calibration(object):
    def __init__(self, robot_frame, eef_frame, camera_frame, marker_frame, base_marker_frame=None, abs_range_pos=4, abs_range_rot=1):
        self.robot_frame = robot_frame
        self.eef_frame = eef_frame
        self.camera_frame = camera_frame
        self.marker_frame = marker_frame
        self.is_mobile = not base_marker_frame is None
        self.base_marker_frame = base_marker_frame
        self.bounds = self.init_bounds(abs_range_pos, abs_range_rot)
        self.tfl = tf.TransformListener(True, rospy.Duration(2))  # tf will have 2 seconds of cache
    
        rospack = rospkg.RosPack()
        self.conf_dir = join(rospack.get_path("robot_calibration"), "config")
        if not exists(self.conf_dir):
            makedirs(self.conf_dir)

    def init_bounds(self, abs_range_pos, abs_range_rot):
        bounds = []
        pos_bounds = [-abs_range_pos, abs_range_pos]
        rot_bounds = [-abs_range_rot, abs_range_rot]
        for i in range(2):
            for j in range(3):
                bounds.append(pos_bounds)
            for j in range(4):
                bounds.append(rot_bounds)
        return bounds

    def record_calibration_points(self, continuous = True, duration=60, min_dist=0.01, max_dur=0.05):
        mat_robot = [] # Matrix of all calibration points of eef_frame in robot_frame
        mat_camera = [] # Matrix of all calibration points of marker_frame in frame camera_frame
        mat_mobile_base = [] # Matrix of all calibration points of marker_frame in frame camera_frame
        max_dur = rospy.Duration(max_dur) # seconds
        duration = rospy.Duration(duration)
        
        start = rospy.Time.now()
        last_point = None
        entry = ""
        while continuous and rospy.Time.now()<start+duration or not continuous and entry=="":   
            try:
                pose_rg_robot = self.tfl.lookupTransform(self.robot_frame, self.eef_frame, rospy.Time(0))
            except Exception, e:
                print("Robot <-> Camera transformation not available at the last known common time:", e.message)              
            else:
                if last_point is None or transformations.distance(pose_rg_robot, last_point)>min_dist:
                    try:
                        pose_rg_opt = self.tfl.lookupTransform(self.camera_frame, self.marker_frame, rospy.Time(0))
                    except:
                        print("Marker not visible at the last know common time")
                    else:
                        mat_robot.append(np.array(pose_rg_robot))
                        mat_camera.append(np.array(pose_rg_opt))
                        last_point = pose_rg_robot
                        if self.is_mobile:
                            try:
                                pose_rg_base = self.tfl.lookupTransform(self.camera_frame, self.base_marker_frame, rospy.Time(0))
                            except Exception, e:
                                print("Base <-> Camera transformation not available at the last known common time:", e.message)
                            else:
                                mat_mobile_base.append(np.array(pose_rg_base))
                        print("pose recorded")
            
            if continuous:
                rospy.sleep(0.25)
            else:
                entry = raw_input("Press enter to record a new point or q-enter to quit ({} points)".format(len(mat_robot)))
        return mat_camera, mat_robot, mat_mobile_base

    @staticmethod
    def extract_transforms(flat_transforms):
        # a transform is 3 pos and 4 rot
        nb_transform = len(flat_transforms) / 7
        list_transforms = []
        for i in range(nb_transform):
            pose = []
            # extract the pose
            pose.append(flat_transforms[i * 7:i * 7 + 3])
            pose.append(flat_transforms[i * 7 + 3:i * 7 + 7])
            # append it to the list of transforms
            list_transforms.append(pose)
        return list_transforms
        
    @staticmethod
    def result_to_calibration_matrix(result):
        calibration_matrix = transformations.inverse_transform(result)
        return [map(float, calibration_matrix[0]), map(float, calibration_matrix[1].tolist())]

    @staticmethod
    def evaluate_calibration(calibrations, coords_robot, coords_opt, coords_base):
        def quaternion_cost(norm_coeff):
            C = 0
            for transform in list_calibr:
                # norm of a quaternion is always 1
                C += norm_coeff * abs(np.linalg.norm(transform[1]) - 1)
            return C
        def distance_cost(pose1, pose2, rot_coeff=2):
            pos_cost = 0
            # calculate position ditance
            pos_cost = np.linalg.norm(np.array(pose1[0]) - np.array(pose2[0]))
            # distance between two quaternions
            rot_cost = 1 - np.inner(pose1[1], pose2[1])**2
            return pos_cost + rot_coeff * rot_cost
        # first extract the transformations
        list_calibr = Calibration.extract_transforms(calibrations)
        # set the base transform
        A = list_calibr[0]
        B = list_calibr[1]
        # loop trough all the transforms
        cost = quaternion_cost(1)
        nb_points = len(coords_robot)
        for i in range(nb_points):
            robot = coords_robot[i]
            opt = coords_opt[i]
            product = transformations.multiply_transform(robot, B)
            product = transformations.multiply_transform(A, product)
            if self.is_mobile:
                mobile_base = coords_base[i]
                product = transformations.multiply_transform(mobile_base, product)
            product[1] /= np.linalg.norm(product[1])
            cost += distance_cost(opt, product)
        return cost

    def calibrate(self):
        raw_input("Press enter to start the recording process")
        # Record during 60 sec... set continuous=False for an interactive mode
        mat_camera, mat_robot, mat_mobile_base = self.record_calibration_points(continuous=True)
        initial_guess = [0,0,0,0,0,0,1]*2 # have to estimate two full rotation matrices
        print("Recording finished, calibrating, please wait.")
        t0 = time.time()
        # Be patient, this cell can be long to execute...
        result = minimize(self.evaluate_calibration, initial_guess, args=(mat_robot, mat_camera, mat_mobile_base),
                          method='L-BFGS-B', bounds=self.bounds)
        print time.time()-t0, "seconds of optimization"
        result_list = self.extract_transforms(result.x)

        calibration_matrix_a = self.result_to_calibration_matrix(result_list[0])
        calibration_matrix_b = self.result_to_calibration_matrix(result_list[1])

        if exists(join(self.conf_dir, "calibration_matrices.yml")):
            with open(join(self.conf_dir, "calibration_matrices.yml"), 'r') as f:
                calibrations = yaml.load(f)
        else:
            calibrations = {}
        
        if self.is_mobile:
            key = self.base_marker_frame + "-" + self.robot_frame
            calibrations[key] = calibration_matrix_a
            key = self.eef_frame + "- " + self.marker_frame
            calibrations[key] = calibration_matrix_b
        else:
            key = self.camera_frame + "-" + self.robot_frame
            calibrations[key] = calibration_matrix_a
            key = self.eef_frame + "-" + self.marker_frame
            calibrations[key] = calibration_matrix_b

        with open(join(self.conf_dir, "calibration_matrices.yml"), 'w') as f:
            yaml.dump(calibration, f)
        print("Calibration complete")