#!/usr/bin/env python3

import rospy
import math
import rosparam
import argparse
import time

from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
#from posegrab import *
from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import JointState





import numpy as np




class BaseController:
    def __init__(self, activate: bool = False):


        try:
            rospy.init_node('controller')
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
        
        self.set_angles = [1.5]*6
        self.sub = rospy.Subscriber("pose_estimate",Float64MultiArray,self.angles_callback)
        
        # read config files
        self.is_activated = activate
  
        self.__velocity_publisher = rospy.Publisher("gen3_lite/joint_group_position_controller/command",
                                                    Float64MultiArray, queue_size=10)

        self.state_sub = rospy.Subscriber("/my_gen3_lite/joint_states",JointState,self.state_callback)

        self.command = [0]*6
        
        
        
        self.HOME_ACTION_IDENTIFIER = 2
        self.last_action_notif_type = None

        try:
            self.robot_name = "gen3_lite"#rospy.get_param('~robot_name')
            rospy.wait_for_service('my_gen3_lite/base/send_joint_speeds_command')
            self.move = rospy.ServiceProxy('my_gen3_lite/base/send_joint_speeds_command', SendJointSpeedsCommand)

            rospy.wait_for_service('my_gen3_lite/base/get_measured_joint_angles')
            self.get_angles = rospy.ServiceProxy('my_gen3_lite/base/get_measured_joint_angles',GetMeasuredJointAngles)
            
            #self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)

            # Wait for the driver to be initialised
            # while not rospy.has_param("/is_initialized"):
            #     time.sleep(0.1)
            
            # # Init the services
            # read_action_full_name = '/' + self.robot_name + '/base/read_action'
            # rospy.wait_for_service(read_action_full_name)
            # self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            # execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            # rospy.wait_for_service(execute_action_full_name)
            # self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)
        except rospy.ROSException as e:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def state_callback(self,data):
        P = 1
        D = 0

        delta = 0.4

        angles = data.position[0:-1]
        errors = [self.set_angles[i] - angles[i] for i in range(len(self.set_angles))]
        velocities = data.velocity[0:-1]
        self.command = [(P*errors[i] if(abs(errors[i])> delta) else 0) - D*velocities[i] for i in range(len(velocities))]
        self.command[3:] = [0,0,0]
        print(self.command)


    
    def move_to_joint_angles(self,speeds):
        #print(speeds)
        
        try:
            
            speeds_msg = Base_JointSpeeds()
            speeds_msg.joint_speeds = speeds
            response = self.move(speeds_msg)
            print(response)
            time.sleep(2)
        except Exception as e:
            print(e)
            return

    def angles_callback(self,data):
        #print(data.data)
        self.set_angles = data.data


    def publish_joints(self):
        joints = []
        for i in range(len(self.command)):
            joint = JointSpeed()
            joint.joint_identifier = i + 1
            joint.value = self.command[i]
            joints.append(joint)
        self.move_to_joint_angles(joints)
        # dim = MultiArrayDimension()
        # dim.label = "angles"
        # dim.size = 6
        # dim.stride = 1
        # layout = MultiArrayLayout([dim],0)
        # self.__velocity_publisher.publish(Float64MultiArray(layout,self.angles))\




def main():
    rospy.init_node("base_controller", anonymous=True)
    bc_module = BaseController(True)

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown() and bc_module.is_activated:
        #bc_module.set_command()
        bc_module.publish_joints()
        rate.sleep()
    rospy.signal_shutdown("Controller served its purpose.")


if __name__ == '__main__':
    try:
        rospy.sleep(12)
        main()
    except rospy.exceptions.ROSInterruptException:
        rospy.logerr(f'Shutting down peacefully due to a user interrupt.')
    else:
        rospy.logerr(f'Shutting down...')