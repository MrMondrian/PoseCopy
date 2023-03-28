#!/usr/bin/env python3

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.python.solutions.pose import PoseLandmark
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import math

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
                #print("hi")
                success, image = self.cap.read()
                #success, image = self.cap.read()
                #print(success)
                if not success:
                    #print("Fail")
                    # If loading a video, use 'break' instead of 'continue'.
                    yield None

                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = pose.process(image)
                if(results and results.pose_world_landmarks):
                    yield [self.shoulder_rotation(results),abs(3.14 -self.shoulder_angle(results)),abs(3.14 - self.elbow_angle(results)),0,0,0]
                else:
                    yield [1.5]*6
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


def pub_angles(angles):
    dim = MultiArrayDimension()
    dim.label = "angles"
    dim.size = 6
    dim.stride = 1
    layout = MultiArrayLayout([dim],0)
    pub.publish(Float64MultiArray(layout,angles))

if __name__ == '__main__':
    rospy.init_node("pose_estimation",anonymous=True)
    pub = rospy.Publisher("pose_estimate",Float64MultiArray,queue_size=10)


    


    #posecap = PoseCapture(video='/home/anthony/comp400/sim/kinova-arm/catkin_ws/src/base_controller/scripts/test.mp4')
    posecap = PoseCapture()
    #fps = posecap.cap.get(cv2.CAP_PROP_FPS)
    #print("Frame rate: {}".format(fps))
    #rate = rospy.Rate(int(fps))
    rate = rospy.Rate(60)
    
    angle_gen = posecap.angles()

    pub_angles([1.5]*6)
    rospy.sleep(5)

    angles = next(angle_gen,None)
    while not angles == None:
        pub_angles(angles)
        rate.sleep()
        angles = next(angle_gen,None)

    rospy.spin()