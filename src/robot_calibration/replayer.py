#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import JointState
from os.path import join
from os.path import exists
import json


class Replayer(object):
    def __init__(self, joint_state_channel):
        rospack = rospkg.RosPack()
        self.data_dir = join(rospack.get_path("robot_calibration"), "data")
        self.replay = False
        self.pub = rospy.Publisher(joint_state_channel, JointState, queue_size=10)

    def start_replaying(self, savefile, hz=10):
        savefile = join(self.data_dir, savefile + ".json")
        rate = rospy.Rate(hz)
        with open(savefile, 'r') as f:
            data = json.load(f)

        for timestep in data:
            if rospy.is_shutdown(): return
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = timestep["name"]
            msg.position = timestep["position"]
            msg.velocity = timestep["velocity"]
            msg.effort = timestep["effort"]
            self.pub.publish(msg)
            rate.sleep()

    def record_exists(self, savefile):
        savefile = join(self.data_dir, savefile + ".json")
        return exists(savefile)

    def get_first_state(self, savefile):
        savefile = join(self.data_dir, savefile + ".json")
        with open(savefile, 'r') as f:
            data = json.load(f)
        return data[0]