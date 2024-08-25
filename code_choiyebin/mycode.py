#! /usr/bin/env python
# -*- coding: utf-8 -*-
# limo_application/scripts/lane_detection/crosswalk_detect.py
# WeGo LIMO Pro를 이용한 횡단보도 인식 코드

import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from limo_application.cfg import crosswalkConfig

import cv2
import numpy as np

class CrossWalkDetector:
    def __init__(self):
               # ROS Part
        rospy.init_node("crosswalk_detect")
        srv = Server(crosswalkConfig, self.reconfigure_callback)
        self.cvbridge = CvBridge()
        rospy.Subscriber(rospy.get_param("~image_topic_name", "/camera/rgb/image_raw/compressed"), CompressedImage, self.Image_CB)
        self.distance_pub = rospy.Publisher("/limo/crosswalk_y", Int32, queue_size=5)
        self.cross_state_pub = rospy.Publisher("/limo/cross_state", Int32, queue_size=5)  # 상태 퍼블리셔 추가
        self.viz = rospy.get_param("~visualization", True)
        self.cross_state = 1  # 횡단보도 상태 추적 변수
        self.last_cross_time = rospy.Time.now().to_sec()  # 마지막 상태 업데이트 시간

    # return Int32
    def calcCrossWalkDistance(self, _img):
        '''
            최종 검출된 Binary 이미지를 이용하여, 횡단 보도의 모멘트 계산
            모멘트의 x, y 좌표 중 차량과의 거리에 해당하는 y를 반환
        '''
        if self.line_num >= self.CROSS_WALK_DETECT_TH:
            try:

                M = cv2.moments(_img)
                self.x = int(M['m10']/M['m00'])
                self.y = int(M['m01']/M['m00'])
            except:

                self.x = -1
                self.y = -1
            # print("x, y = {}, {}".format(x, y))
            return self.y
        else:

            self.x = 0
            self.y = 0
            return -1

    def visResult(self):
        '''
            최종 결과가 추가된 원본 이미지 (crosswalk_original)
            횡단 보도 영역만 ROI로 잘라낸 이미지 (crosswalk_cropped)
            ROI 내부 중 특정 색 영역만 검출한 이미지 (crosswalk_threshold)
            검출된 이진 이미지의 경계만 검출한 이미지 (crosswalk_edge)
        '''
        if not self.x <= 0 and not self.y <= 0:
            cv2.line(self.cropped_image, (0, self.y), (self.crop_size_x, self.y), (0, 255, 255), 20)

        # cv2.circle(self.cropped_image, (self.x, self.y), 10, 255, -1)
        cv2.imshow("crosswalk_original", self.frame)
        cv2.imshow("crosswalk_cropped", self.cropped_image)
        cv2.imshow("crosswalk_thresholded", self.thresholded_image)
        cv2.imshow("crosswalk_edge", self.edge_image)
#         # cv2.imshow("lines", self.line_detect_image)
        cv2.waitKey(1)

    # return opencv Image type
    def imageCrop(self, _img=np.ndarray(shape=(480, 640))):
        '''
            원하는 이미지 영역 검출
        '''
        self.crop_size_x = 360
        self.crop_size_y = 60
        return _img[400:480, 240:400]

    # return opencv Image type
    def edgeDetect(self, _img=np.ndarray(shape=(480, 640))):
        '''
            이미지의 경계면 검출
        '''
        return cv2.Canny(_img, 0, 360)

    def houghLineDetect(self, _img=np.ndarray(shape=(480, 640))):
        '''
            이미지의 직선 검출
        '''
        new_img = _img.copy()
	try:
            self.lines = cv2.HoughLinesP(new_img, self.RHO, self.THETA * np.pi / 180, self.THRESHOLD, minLineLength=10, maxLineGap=5)
	except:
	    self.lines= None

        if self.lines is None:
            # print("length is 0")
            self.line_num = 0
        else:
#             print("length is {}".format(len(self.lines)))
# 	    print("line_num is {}".format(self.line_num))
            self.line_num = len(self.lines)
            for i in range(self.lines.shape[0]):
                pt1 = (self.lines[i][0][0], self.lines[i][0][1])
                pt2 = (self.lines[i][0][2], self.lines[i][0][3])    
                cv2.line(self.cropped_image, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)

    # return opencv Image type
    def colorDetect(self, _img=np.ndarray(shape=(480, 640))):
        '''
            원하는 색 영역만 추출 (Dynamic Reconfigure를 통해, 값 변경 가능)
        '''

        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_white = cv2.inRange(hls, self.WHITE_LANE_LOW, self.WHITE_LANE_HIGH)

        return mask_white

    # ==============================================
    #               Callback Functions
    # ==============================================

    def reconfigure_callback(self, config, level):
        '''
            Dynamic_Reconfigure를 활용하여, 횡단보도 검출을 위한 색 지정
            HLS Color Space를 기반으로 검출
            흰색 횡단보도 검출을 위한 Threshold 설정
            Hough Transform 기반 직선 검출을 위한 RHO, THETA, THRESHOLD 설정
        '''
        self.WHITE_LANE_LOW = np.array([0, 110, 0])
        self.WHITE_LANE_HIGH = np.array([255, 255, 200])
        self.RHO = float(1.0)
        self.THETA = 1
        self.THRESHOLD = 55
        self.CROSS_WALK_DETECT_TH = 2
        return config

    def Image_CB(self, img):

       self.frame = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")
       self.cropped_image = self.imageCrop(self.frame)
       self.thresholded_image = self.colorDetect(self.cropped_image)
       self.edge_image = self.edgeDetect(self.thresholded_image)
       self.houghLineDetect(self.edge_image)
       self.crosswalk_distance = self.calcCrossWalkDistance(self.thresholded_image)
       self.distance_pub.publish(self.crosswalk_distance)


#     if self.line_num > 0:
#         horizontal_lines = 0
#         for line in self.lines:
#             x1, y1, x2, y2 = line[0]
#             angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
#             rospy.loginfo("Detected line with angle: {}".format(angle))
#             if 80 <= abs(angle) <= 100:  # 거의 수평인 선들만 카운트
#                 horizontal_lines += 1
#
#          rospy.loginfo("Number of horizontal lines:{}".format(horizontal_lines))
#
#
#         if horizontal_lines > 0 and self.y > 0:
#             current_time = rospy.Time.now().to_sec()
#             if current_time - self.last_cross_time > 5.0:  # 딜레이 타임 5초
#                 rospy.loginfo("Crosswalk detect!")
#                 self.cross_state += 1
#                 rospy.loginfo("Cross State Updated: {}".format(self.cross_state))
#                 self.cross_state_pub.publish(self.cross_state)
#                 self.last_cross_time = current_time

       if self.cross_state == 4:
           rospy.sleep(5)

       elif self.cross_state > 6:
           self.cross_state = 1

       if self.x > 0 and self.y > 0:
           current_time = rospy.Time.now().to_sec()
           if current_time - self.last_cross_time > 5.0:  # 딜레이 타임 5초
               rospy.loginfo("Crosswalk detect!")
               self.cross_state += 1
               rospy.loginfo("Cross State Updated: {}".format(self.cross_state))
               rospy.sleep(1.5)
               self.cross_state_pub.publish(self.cross_state)
               self.last_cross_time = current_time


       # visualization
        if self.viz:
            self.visResult()


def run():
    new_class = CrossWalkDetector()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")

