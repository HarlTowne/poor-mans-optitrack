import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from marker_msgs.msg import MatchedMarkers
from cv_bridge import CvBridge
from math import sqrt, pi, e
from typing import List
import astroalign as aa
import timeit
import time

import cv2
import numpy as np
POINT_MATCH_MARGIN = 10

class BrightnessThresholder(Node):

    def __init__(self):
        super().__init__('brightness_thresholder')
        self._left_img_sub = self.create_subscription(Image, 'left/image_rect', self._recv_left, 1)
        self._right_img_sub = self.create_subscription(Image, 'right/image_rect', self._recv_right, 1)
        self._out_pub = self.create_publisher(MatchedMarkers, 'matched_markers', 1)
        self.declare_parameter("threshold", 240)

        self.last_left_points = []
        self.last_right_points = []

        self.left_img = None
        self.right_img = None
        self.n = 0
        self._bridge = CvBridge()

    def _recv_left(self, msg: Image):
        cv_image = self._bridge.imgmsg_to_cv2(msg)
        points = self._get_points_from_img(cv_image)
        entry = {"header": msg.header, "points": points}
        self.last_left_points.append(entry)

        try:
            self._match_frames()
        except Exception as e:
            print(e)
        colour = cv2.cvtColor(cv_image.copy(), cv2.COLOR_GRAY2BGR)
        self.left_img = colour.copy()

    def _recv_right(self, msg: Image):
        cv_image = self._bridge.imgmsg_to_cv2(msg)
        points = self._get_points_from_img(cv_image)
        entry = {"header": msg.header, "points": points}
        self.last_right_points.append(entry)

        try:
            self._match_frames()
        except Exception as e:
            print(e)
        colour = cv2.cvtColor(cv_image.copy(), cv2.COLOR_GRAY2BGR)
        self.right_img = colour.copy()

    def _match_frames(self):
        cv2.waitKey(1) 
        if len(self.last_left_points) == 0:
            return
        if len(self.last_right_points) == 0:
            return
        if self.last_right_points[0]["header"].frame_id > self.last_left_points[0]["header"].frame_id:
            self.last_left_points.pop(0)
            return

        for i, right_entry in enumerate(self.last_right_points):
            if self.last_left_points[0]["header"].frame_id == right_entry["header"].frame_id:
                try:
                    self._match_markers(self.last_left_points[0]["points"], right_entry["points"], right_entry["header"])
                except Exception as e:
                    print(e)
                self.last_left_points.pop(0)
                self.last_right_points.pop(i)
                if i != 0:
                    self.last_right_points.pop(0)
        
    def _match_markers(self, left_points: List, right_points: List, header):
        if len(left_points) == 0 or len(right_points) == 0:
            return
        np_left_points = np.int32(left_points)
        np_right_points = np.int32(right_points)
        sort_left_points = np.lexsort((np_left_points[:,0], np_left_points[:,1]))    
        sort_right_points = np.lexsort((np_right_points[:,0], np_right_points[:,1]))
        sort_left_points = np_left_points[sort_left_points] 
        sort_right_points = np_right_points[sort_right_points] 
        # print("===========================================")
        # print(sort_left_points, "\n")
        # print(sort_right_points)

        # print("-------------------------------------------")
        # start = time.process_time()
        # try:
        #     transf, (s_list, t_list) = aa.find_transform(sort_left_points, sort_right_points)
        #     end = time.process_time()
        #     print(end-start)
        # except Exception as ee:
        #     print(ee)
        #     self.n += 1
        #     print("error", self.n)
        #     return
        # print(*[str(s_list[i]) + "==" + str(t_list[i]) + "\n" for i in range(len(s_list))])
        matched_left: List[Point] = []#[Point(x=float(s_list[i][0]), y=float(s_list[i][1]), z = float(0)) for i in range(len(s_list))]
        matched_right: List[Point] = []#[Point(x=float(t_list[i][0]), y=float(t_list[i][1]), z = float(0)) for i in range(len(t_list))]
        # print(matched_left)
        # print(matched_right)

        c = 1
        # for i in range(len(s_list)):
        #     colour = cv2.cvtColor(np.uint8([[[360/e*c, 255, 255]]]), cv2.COLOR_HSV2BGR)
        #     colour = tuple(colour[0][0])
        #     colour = ( int (colour [ 0 ]), int (colour [ 1 ]), int (colour [ 2 ])) 
        #     cv2.circle(self.left_img, s_list[i], POINT_MATCH_MARGIN//2, colour, cv2.FILLED)
        #     cv2.circle(self.right_img, t_list[i], POINT_MATCH_MARGIN//2, colour, cv2.FILLED)
        #     c += 1

        for left_point in sort_left_points:
            for i, right_point in enumerate(sort_right_points):
                if abs(left_point[1] - right_point[1]) < POINT_MATCH_MARGIN:
                    colour = cv2.cvtColor(np.uint8([[[360/e*c, 255, 255]]]), cv2.COLOR_HSV2BGR)
                    colour = tuple(colour[0][0])
                    colour = ( int (colour [ 0 ]), int (colour [ 1 ]), int (colour [ 2 ])) 
                    cv2.circle(self.left_img, left_point, POINT_MATCH_MARGIN//2, colour, cv2.FILLED)
                    cv2.circle(self.right_img, right_point, POINT_MATCH_MARGIN//2, colour, cv2.FILLED)
                    matched_left.append(Point(x=float(left_point[0]), y=float(left_point[1]), z = float(0)))
                    matched_right.append(Point(x=float(right_point[0]), y=float(right_point[1]), z = float(0.0)))
                    sort_right_points = np.delete(sort_right_points, i, 0)
                    c += 1
                    break
        out = MatchedMarkers()
        out.header = header
        out.left = matched_left
        out.right = matched_right
        self._out_pub.publish(out)
        cv2.imshow("right", self.right_img)
        cv2.imshow("left", self.left_img)
        cv2.waitKey(1) 
        


            

            

    def _get_points_from_img(self, image):
        threshold = self.get_parameter('threshold').get_parameter_value().integer_value
        thresholded = cv2.inRange(image, threshold, 255)
        contours = cv2.findContours(thresholded.copy(),
                                cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        points = []
        for contour in contours:
            M = cv2.moments(contour)
            m10 = M["m10"]
            m00 = M["m00"]
            m01 = M["m01"]
            if m00 == 0:
                m00 = 0.0001
            x = int(m10/m00)
            y = int(m01/m00)
            points.append((x, y))
            
        return points



def main(args = None):
    rclpy.init(args=args)
    marker_detector = BrightnessThresholder()
    try:
        rclpy.spin(marker_detector)
    except:
        pass
    marker_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
