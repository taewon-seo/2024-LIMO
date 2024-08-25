#!/usr/bin/env python
# -*- coding:utf-8 -*-
# limo_application/scripts/lane_detection/control.py
# WeGo LIMO Pro를 이용한 주행 코드

import rospy
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
from object_msgs.msg import ObjectArray
from limo_base.msg import LimoStatus
from dynamic_reconfigure.server import Server
from limo_application.cfg import controlConfig

import math
import time


class LimoController:
    '''
        차선 인식, 횡단보도 인식, LiDAR 기반 장애물 인식, YOLO 기반 신호등 및 표지판 인식
        위 기능을 통합한 전체 주행 코드
        Private Params --> control_topic_name
        < Subscriber >
        limo_status (LimoStatus) --> LIMO의 Motion Model 확인용
        /limo/lane_x (Int32) --> 인식된 차선 (카메라 좌표계 x)
        /limo/crosswalk_y (Int32) --> 인식된 횡단보도 (카메라 좌표계 y)
        < Publisher >
        /cmd_vel (Twist) --> Default 출력, LIMO 제어를 위한 Topic
    '''
    def __init__(self):
        rospy.init_node('limo_control', anonymous=True)

        ##### 리모 기본 설정
        self.LIMO_WHEELBASE = 0.2
        self.distance_to_ref = 0
        self.crosswalk_detected = False
        self.limo_mode = "ackermann"
        self.e_stop = "Safe"


        ###### 미션 관련 변수
        self.mission_flag = 1
        

        srv = Server(controlConfig, self.reconfigure_callback)
        rospy.Subscriber("limo_status", LimoStatus, self.limo_status_callback)
        rospy.Subscriber("/limo/lane_x", Int32, self.lane_x_callback)

        rospy.Subscriber("/limo/lidar_warning", String, self.lidar_warning_callback)

        rospy.Subscriber("/limo/cross_state", Int32, self.mission_flag_callback)



        self.drive_pub = rospy.Publisher(rospy.get_param("~control_topic_name", "/cmd_vel"), Twist, queue_size=1)
        
        ############# 주기 설정
        rospy.Timer(rospy.Duration(0.03), self.drive_callback)




    # return float
    def calcTimeFromDetection(self, _last_detected_time):
        '''
            마지막 검출 시간부터 흐른 시간 확인하는 함수
        '''
        return rospy.Time.now().to_sec() - _last_detected_time


    def limo_status_callback(self, _data):
        if _data.motion_mode == 1:
            if self.limo_mode == "ackermann":
                pass
            else:
                self.limo_mode = "ackermann"
                # rospy.loginfo("Mode Changed --> Ackermann")
        else:
            if self.limo_mode == "diff":
                pass
            else:
                self.limo_mode = "diff"
                # rospy.loginfo("Mode Changed --> Differential Drive")


    def lidar_warning_callback(self, _data):
        '''
            장애물 유무 저장
        '''
        self.e_stop = _data.data

    
    def lane_x_callback(self, _data):
        '''
            실제 기준 좌표와 검출된 차선과의 거리 저장
        '''
        if _data.data == -1:
            self.distance_to_ref = 0
        else:
            self.distance_to_ref = self.REF_X - _data.data



    def mission_flag_callback(self, _data):
        self.mission_flag_1 = _data
        rospy.loginfo("mission_flag: {}".format(self.mission_flag))



#####################################################################################################
## parameter ##

    def reconfigure_callback(self, _config, _level):
        '''
            Dynamic_Reconfigure를 활용
            차량 제어 속도 (BASE_SPEED)
            횡방향 제어 Gain (LATERAL_GAIN)
            차선 기준 좌표 (카메라 픽셀 좌표계 기준) (REF_X)
        '''

        self.BASE_SPEED = 0.3
        self.LATERAL_GAIN = float(0.01)

        ##### 오른쪽 차선 붙는 정도
        self.REF_X = 270        


        return _config

#####################################################################################################


    



    def drive_callback(self, _event):
        '''
            입력된 데이터를 종합하여,
            속도 및 조향을 조절하여 최종 cmd_vel에 Publish
        '''


        ##### 주행 미션 별 분기 수행

        drive_data = Twist()

        if self.mission_flag == 1:

            rospy.loginfo("mission_1")
            
            drive_data.linear.x = 0.4
            self.LATERAL_GAIN = float(0.035)
            self.REF_X = 270

            drive_data.angular.z = self.distance_to_ref * self.LATERAL_GAIN

            if self.distance_to_ref > 15:
                drive_data.angular.z = 0.7


            elif self.distance_to_ref == 0:

                drive_data.linear.x = 0.1
                drive_data.angular.z = 0.3


        elif self.mission_flag == 2:

            rospy.loginfo("mission_2")
            
            drive_data.linear.x = 0.4
            self.LATERAL_GAIN = float(0.035)
            self.REF_X = 270

            drive_data.angular.z = self.distance_to_ref * self.LATERAL_GAIN

            if self.distance_to_ref > 15:
                drive_data.angular.z = 0.7

            elif self.distance_to_ref == 0:

                drive_data.linear.x = 0.1
                drive_data.angular.z = 0.3


        elif self.mission_flag == 3:
            
            rospy.loginfo("mission_3")
            
            
            self.LATERAL_GAIN = float(0.035)
            self.REF_X = 270

            # drive_data.angular.z = self.distance_to_ref * self.LATERAL_GAIN

            drive_data.linear.x =  0.3
            drive_data.angular.z = 0.0

            ##### 장애물 발견 시 후진, 우회전 시나리오 구성
            #     

            if self.e_stop == "Warning":

                rospy.logwarn("Obstacle Detected, Stop!")

                ## 장애물 인식 멈춤
                time.sleep(2)

                drive_data.linear.x = -0.3
                drive_data.angular.z = 0.0

                ## 후진
                self.pub_cmd_vel(drive_data)
                rospy.loginfo(drive_data)
                time.sleep(0.5)

                self.pub_cmd_vel(drive_data)
                time.sleep(0.5)

                self.pub_cmd_vel(drive_data)
                time.sleep(0.5)

                ## 후진 후 멈춤
                time.sleep(2)

                ## 핸들 틀고 주행
                drive_data.linear.x = 0.3
                drive_data.angular.z = 1.0


                self.pub_cmd_vel(drive_data)
                time.sleep(0.5)

                self.pub_cmd_vel(drive_data)
                time.sleep(0.5)

                drive_data.linear.x = 0.3
                drive_data.angular.z = -1.0

                self.pub_cmd_vel(drive_data)
                time.sleep(0.5)

                self.pub_cmd_vel(drive_data)
                time.sleep(0.5)

                
                # self.my_time = rospy.Time.now().to_sec()
                # if rospy.Time.now().to_sec() - self.my_time.to_sec() < 3.0 :

        elif self.mission_flag == 4:

            drive_data.linear.x = 0.3
            self.LATERAL_GAIN = float(0.020)
            self.REF_X = 210

            drive_data.angular.z = self.distance_to_ref * self.LATERAL_GAIN
            if drive_data.angular.z > 0.7:
                drive_data.angular.z = 0.7
        

        self.pub_cmd_vel(drive_data)


        


        
    def pub_cmd_vel(self, _data):
        
        drive_data = Twist()
        drive_data = _data

        ##### 현재 angular.z 출력
        #rospy.loginfo("OFF_CENTER, value z = {}, {}".format(self.distance_to_ref, drive_data.angular.z))


        ##### 리모 속도 설정 및 조향 값 퍼블리시 
        try:

            if self.limo_mode == "diff":
                self.drive_pub.publish(drive_data)

            elif self.limo_mode == "ackermann":


                rospy.loginfo("REF{}".format(self.REF_X))

                drive_data.angular.z = \
                    math.tan(drive_data.angular.z / 2) * drive_data.linear.x / self.LIMO_WHEELBASE
                # 2를 나눈 것은 Differential과 GAIN비율을 맞추기 위함

                

                self.drive_pub.publish(drive_data)

        except Exception as e:
            rospy.logwarn(e)


    

def run():
    new_class = LimoController()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
