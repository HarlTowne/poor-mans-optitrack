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
from statistics import median

import cv2
import numpy as np
POINT_MATCH_MARGIN = 30

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
        # print("recvleft")

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
        # print("recvright")

        try:
            self._match_frames()
        except Exception as e:
            print(e)
        colour = cv2.cvtColor(cv_image.copy(), cv2.COLOR_GRAY2BGR)
        self.right_img = colour.copy()

    def _match_frames(self):
        cv2.waitKey(1) 
        # print("matching")
        # print(len(self.last_left_points), len(self.last_right_points))
        if len(self.last_left_points) == 0:
            return
        if len(self.last_right_points) == 0:
            return
        if self.last_right_points[0]["header"].frame_id > self.last_left_points[0]["header"].frame_id:
            self.last_left_points.pop(0)
            return
        # print("leftid:", self.last_left_points[0]["header"].frame_id)
        # print("rightid:", self.last_right_points[0]["header"].frame_id)
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
            if i > 3:
                # self.last_right_points.pop(0)
                print("emergency pop")
                self.last_left_points.pop(0)
                return

        
    def _match_markers(self, left_points: List, right_points: List, header):
        if len(left_points) == 0 or len(right_points) == 0:
            return
        # print("#########################################")
        # print(len(left_points))
        np_left_points = np.int32(left_points)
        np_right_points = np.int32(right_points)
        sort_left_points = np.lexsort((np_left_points[:,1], np_left_points[:,0]))    
        sort_right_points = np.lexsort((np_right_points[:,1], np_right_points[:,0]))
        sort_left_points = np_left_points[sort_left_points] 
        sort_right_points = np_right_points[sort_right_points] 
        
        median_x_left = median([sort_left_points[i][0] for i in range(len(left_points))])
        median_x_right = median([sort_right_points[i][0] for i in range(len(right_points))])

        median_diff = median_x_left - median_x_right
        # print(median_diff)
        for point in sort_left_points:
            cv2.circle(self.left_img, point, 2, [0, 0, 255], cv2.FILLED)

        for point in sort_right_points:
            cv2.circle(self.right_img, point, 2, [0, 0, 255], cv2.FILLED)

        matched_left: List[Point] = []
        matched_right: List[Point] = []

        c = 1
        for left_point in sort_left_points:
            closest_point = [999999, 999999]
            closest_index = -1
            closest_dist = 999999999
            # print(left_point)
            for i, right_point in enumerate(sort_right_points):
                # print(i)
                x_diff = left_point[0]-right_point[0]-median_diff
                y_diff = left_point[1]-right_point[1]
                dist = sqrt(x_diff*x_diff + y_diff*y_diff)
                if dist < closest_dist:
                    closest_point = right_point
                    closest_index = i
                    closest_dist = dist
                    # print(closest_point, closest_index)

            if closest_dist < POINT_MATCH_MARGIN:
                colour = cv2.cvtColor(np.uint8([[[360/e*c, 255, 255]]]), cv2.COLOR_HSV2BGR)
                colour = tuple(colour[0][0])
                colour = ( int (colour [ 0 ]), int (colour [ 1 ]), int (colour [ 2 ])) 
                # print([left_point[0]-median_diff, left_point[1]], [closest_point[0]+median_diff, closest_point[1]])
                # cv2.circle(self.right_img, [int(left_point[0]-median_diff), left_point[1]], POINT_MATCH_MARGIN//2, colour, cv2.FILLED)
                # cv2.circle(self.left_img, [int(closest_point[0]+median_diff), closest_point[1]], POINT_MATCH_MARGIN//2, colour, cv2.FILLED)
                cv2.circle(self.left_img, left_point, POINT_MATCH_MARGIN//2, colour, cv2.FILLED)
                cv2.circle(self.right_img, closest_point, POINT_MATCH_MARGIN//2, colour, cv2.FILLED)
                c += 1
                # print(closest_point, closest_index)
                matched_left.append(Point(x=float(left_point[0]), y=float(left_point[1]), z = float(0)))
                matched_right.append(Point(x=float(closest_point[0]), y=float(closest_point[1]), z = float(0.0)))
                sort_right_points = np.delete(sort_right_points, closest_index, 0)


        # for left_point in sort_left_points:
        #     for i, right_point in enumerate(sort_right_points):
        #         if abs(left_point[1] - right_point[1]) < POINT_MATCH_MARGIN:
        #             colour = cv2.cvtColor(np.uint8([[[360/e*c, 255, 255]]]), cv2.COLOR_HSV2BGR)
        #             colour = tuple(colour[0][0])
        #             colour = ( int (colour [ 0 ]), int (colour [ 1 ]), int (colour [ 2 ])) 
        #             cv2.circle(self.left_img, left_point, POINT_MATCH_MARGIN//2, colour, cv2.FILLED)
        #             cv2.circle(self.right_img, right_point, POINT_MATCH_MARGIN//2, colour, cv2.FILLED)
        #             matched_left.append(Point(x=float(left_point[0]), y=float(left_point[1]), z = float(0)))
        #             matched_right.append(Point(x=float(right_point[0]), y=float(right_point[1]), z = float(0.0)))
        #             sort_right_points = np.delete(sort_right_points, i, 0)
        #             c += 1
        #             break
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
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
