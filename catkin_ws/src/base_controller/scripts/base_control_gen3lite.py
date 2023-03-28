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




#!/usr/bin/env python3

# -*- coding: utf-8 -*-
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.python.solutions.pose import PoseLandmark


mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils 
mp_drawing_styles = mp.solutions.drawing_styles


class PoseCapture:


    def __init__(self,video=None):
        self.DESIRED_HEIGHT = 480
        self.DESIRED_WIDTH = 480
        if video == None:
            self.cap = cv2.VideoCapture(0)
        else:
            self.cap = cv2.VideoCapture(video)


    def resize_and_show(self, image):
        h, w = image.shape[:2]
        if h < w:
            img = cv2.resize(image, (DESIRED_WIDTH, math.floor(h/(w/DESIRED_WIDTH))))
        else:
            img = cv2.resize(image, (math.floor(w/(h/DESIRED_HEIGHT)), DESIRED_HEIGHT))
        cv2.imshow("",img)

    def angle(self, v1,v2,z_scale):
        v1[-1] /= z_scale
        v2[-1] /= z_scale
        dot = sum([v1[i] * v2[i] for i in range(len(v1))])
        norm1 = math.dist([0]*len(v1),v1)
        norm2 = math.dist([0]*len(v1),v2)
        #print(dot/(norm1*norm2))
        return math.acos(dot/(norm1*norm2))


    def landmarks_to_dict(self, landmarks):
        out = []
        # print("##############################")
        # print(landmarks.landmark)
        for (i,lm) in enumerate(landmarks.landmark):
            out.append(lm)
        return out

    def dict_to_list(self, d):
        return [d.x,d.y,d.z]

    def elbow_angle(self, results):
        landmarks = results.pose_world_landmarks
        lm_dict = self.landmarks_to_dict(landmarks)
        wrist = self.dict_to_list(lm_dict[PoseLandmark.RIGHT_WRIST])
        elbow = self.dict_to_list(lm_dict[PoseLandmark.RIGHT_ELBOW])
        shoulder = self.dict_to_list(lm_dict[PoseLandmark.RIGHT_SHOULDER])

        v1 = [shoulder[i] - elbow[i] for i in range(3)]
        v2 = [wrist[i] - elbow[i] for i in range(3)]
        return self.angle(v1,v2,3)
        #return self.angle(v1,v2,30)

    def shoulder_angle(self, results):
        landmarks = results.pose_world_landmarks
        lm_dict = self.landmarks_to_dict(landmarks)
        hip = self.dict_to_list(lm_dict[PoseLandmark.RIGHT_HIP])
        elbow = self.dict_to_list(lm_dict[PoseLandmark.RIGHT_ELBOW])
        shoulder = self.dict_to_list(lm_dict[PoseLandmark.RIGHT_SHOULDER])

        v1 = [elbow[i] - shoulder[i] for i in range(3)]
        v2 = [hip[i] - shoulder[i] for i in range(3)]
        return self.angle(v1,v2,2)
        #return self.angle(v1,v2,15)

    def shoulder_rotation(self, results):
        landmarks = results.pose_world_landmarks
        lm_dict = self.landmarks_to_dict(landmarks)
        elbow = self.dict_to_list(lm_dict[PoseLandmark.RIGHT_ELBOW])
        shoulder = self.dict_to_list(lm_dict[PoseLandmark.RIGHT_SHOULDER])

        v1 = [elbow[0] - shoulder[0], elbow[2] - shoulder[2]]
        v2 = [1,0]
        return self.angle(v1,v2,1)
        #return self.angle(v1,v2,7)



    def angles(self):
        with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=2) as pose:
            while self.cap.isOpened():
                success, image = self.cap.read()
                #print(success)
                if not success:
                    #print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
                    yield None

                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = pose.process(image)
                yield [self.shoulder_rotation(results),abs(3.14 -self.shoulder_angle(results)),abs(3.14 - self.elbow_angle(results)),0,0,0]
            self.cap.release()

    def angle_from_image(self,path):
        with mp_pose.Pose(
        static_image_mode=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=2) as pose:
            image = cv2.imread(path, cv2.IMREAD_COLOR)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image)
            return [self.shoulder_rotation(results),abs(3.14 -self.shoulder_angle(results)),abs(3.14 - self.elbow_angle(results)),0,0,0]
    def move_frame(self,num):
        for i in range(num):
            _,_ = self.cap.read()
        



class BaseController:
    def __init__(self, activate: bool = False):


        try:
            rospy.init_node('controller')
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
        
        self.angles = [1.5]*6
        self.sub = rospy.Subscriber("pose_estimate",Float64MultiArray,self.angles_callback)
        
        # read config files
        self.is_activated = activate
  
        self.__velocity_publisher = rospy.Publisher("gen3_lite/joint_group_position_controller/command",
                                                    Float64MultiArray, queue_size=10)
        
        
        
        self.HOME_ACTION_IDENTIFIER = 2
        self.last_action_notif_type = None

        try:
            self.robot_name = "gen3_lite"#rospy.get_param('~robot_name')
            rospy.wait_for_service('my_gen3_lite/base/send_joint_speeds_command')
            self.move = move = rospy.ServiceProxy('my_gen3_lite/base/send_joint_speeds_command', SendJointSpeedsCommand)
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
        self.angles = data.data


    def publish_joints(self):
        joints = []
        for i in range(len(self.angles)):
            joint = JointSpeed()
            joint.joint_identifier = i + 1
            joint.value = self.angles[i]
            joints.append(joint)
        self.move_to_joint_angles(joints)
        # dim = MultiArrayDimension()
        # dim.label = "angles"
        # dim.size = 6
        # dim.stride = 1
        # layout = MultiArrayLayout([dim],0)
        # self.__velocity_publisher.publish(Float64MultiArray(layout,self.angles))
        


def main():
    rospy.init_node("base_controller", anonymous=True)
    bc_module = BaseController(True)

    rate = rospy.Rate(200)
    
    while not rospy.is_shutdown() and bc_module.is_activated:
        bc_module.publish_joints()
        rate.sleep()
    rospy.signal_shutdown("Controller served its purpose.")


if __name__ == '__main__':
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        rospy.logerr(f'Shutting down peacefully due to a user interrupt.')
    else:
        rospy.logerr(f'Shutting down...')