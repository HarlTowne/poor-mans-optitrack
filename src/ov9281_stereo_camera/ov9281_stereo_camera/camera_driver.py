import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfo_Request, SetCameraInfo_Response
from std_msgs.msg import Header
from cv_bridge import CvBridge

import cv2
import numpy as np
import time
from yaml import safe_load
import os

# ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.04 --ros-args -r right:=/right_camera/image_raw -r left:=/left_camera/image_raw -r left_camera:=/left_camera -r right_camera:=/right_camera
# ros2 run image_view stereo_view --ros-args -r /stereo/left/image:=/left/image_rect -r /stereo/right/image:=/right/image_rect -r /stereo/disparity:=/disparity
class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self._left_img_publiser = self.create_publisher(Image, '/left/image_raw', 1)
        self._right_img_publiser = self.create_publisher(Image, '/right/image_raw', 1)
        self._left_info_publiser = self.create_publisher(CameraInfo, '/left/camera_info', 1)
        self._right_info_publiser = self.create_publisher(CameraInfo, '/right/camera_info', 1)

        self._set_left_info = self.create_service(SetCameraInfo, '/left/set_camera_info', self._test_service)
        self._set_right_info = self.create_service(SetCameraInfo, '/right/set_camera_info', self._test_service)


        self._right_info = CameraInfo()
        self._left_info = CameraInfo()
        try:
            with open(os.path.dirname(os.path.realpath(__file__)) + "/left.yaml") as left_yaml_file:
                left_yaml = safe_load(left_yaml_file)
                self._left_info.width = left_yaml['image_width']
                self._left_info.height = left_yaml['image_height']
                self._left_info.distortion_model = left_yaml['distortion_model']
                self._left_info.d = left_yaml['distortion_coefficients']['data']
                self._left_info.k = left_yaml['camera_matrix']['data']
                self._left_info.r = left_yaml['rectification_matrix']['data']
                self._left_info.p = left_yaml['projection_matrix']['data']

        except:
            pass
        try:
            with open(os.path.dirname(os.path.realpath(__file__)) + "/right.yaml") as right_yaml_file:
                right_yaml = safe_load(right_yaml_file)
                self._right_info.width = right_yaml['image_width']
                self._right_info.height = right_yaml['image_height']
                self._right_info.distortion_model = right_yaml['distortion_model']
                self._right_info.d = right_yaml['distortion_coefficients']['data']
                self._right_info.k = right_yaml['camera_matrix']['data']
                self._right_info.r = right_yaml['rectification_matrix']['data']
                self._right_info.p = right_yaml['projection_matrix']['data']
        except:
            pass

        self._h_rez = 1280
        self._v_rez = 800

        self.declare_parameter("exposure", 4)
        self._exposure = self.get_parameter('exposure').get_parameter_value().integer_value

        self._cap1 = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self._cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap1.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self._cap1.set(cv2.CAP_PROP_EXPOSURE, self._exposure)
        self._cap1.set(cv2.CAP_PROP_FRAME_WIDTH, self._h_rez)
        self._cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, self._v_rez)
        self._cap1.set(cv2.CAP_PROP_FPS, 120)
        self._cap1.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._cap2 = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self._cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap2.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self._cap2.set(cv2.CAP_PROP_EXPOSURE, self._exposure)
        self._cap2.set(cv2.CAP_PROP_FRAME_WIDTH, self._h_rez)
        self._cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, self._v_rez)
        self._cap2.set(cv2.CAP_PROP_FPS, 120)
        self._cap2.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._bridge = CvBridge()

        self._frame_id = 0

        timer_period = 1/120 # seconds
        self.timer = self.create_timer(timer_period, self._publish_from_camera)

    def _test_service(self, request: SetCameraInfo_Request, response: SetCameraInfo_Response):
        print(request)
        print(response)
        response.success = True
        response.status_message = "hi"
        return response

    def _publish_from_camera(self):
        # print("loop", self._frame_id)
        ret1, frame1 = self._cap1.read()
        ret2, frame2 = self._cap2.read()
        exposure = self.get_parameter('exposure').get_parameter_value().integer_value

        if exposure != self._exposure:
            self._exposure = exposure
            self._cap1.set(cv2.CAP_PROP_EXPOSURE, self._exposure)
            self._cap2.set(cv2.CAP_PROP_EXPOSURE, self._exposure)
            

        if not (ret1 and ret2):
            print(self.get_name(), "error getting frames from camera", ret1, ret2)
            return
        
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=str(self._frame_id))
        
        img_msg_left = self._bridge.cv2_to_imgmsg(frame1, encoding="rgb8", header=header)
        img_msg_right = self._bridge.cv2_to_imgmsg(frame2, encoding="rgb8", header=header)
        # info_msg_left = CameraInfo(header=header, width=self._h_rez, height=self._v_rez, distortion_model="plumb_bob", d=[0, 0, 0, 0, 0])
        # info_msg_right = CameraInfo(header=header, width=self._h_rez, height=self._v_rez, distortion_model="plumb_bob", d=[0, 0, 0, 0, 0])
        self._left_info.header = header
        self._right_info.header = header
        self._left_img_publiser.publish(img_msg_left)
        self._right_img_publiser.publish(img_msg_right)
        self._left_info_publiser.publish(self._left_info)
        self._right_info_publiser.publish(self._right_info)

        self._frame_id += 1

    def shutdown(self):

        self._cap1.release()
        self._cap2.release()

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    cam_driver = CameraDriver()
    try:
        rclpy.spin(cam_driver)
    except:
        pass
    cam_driver.shutdown()
    cam_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
