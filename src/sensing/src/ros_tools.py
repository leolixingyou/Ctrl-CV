#!/usr/bin/env python3
import numpy as np

import rospy
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from cv_bridge import CvBridge, CvBridgeError
from collections import deque #deque: Append and pop of elements at both ends are overwhelmingly faster.

CARLA_SENSOR_CONFIG = {
    "Camera": ['/carla/ego_vehicle/rgb_front/image', '/Ctrl_CV/camera/image', Image],
    "LiDAR": ['/carla/ego_vehicle/lidar', '/Ctrl_CV/lidar/pcd', PointCloud2],
    "Gps": ['/carla/ego_vehicle/gnss', '/Ctrl_CV/gps/gnss', NavSatFix],
    "Imu": ['/carla/ego_vehicle/imu', '/Ctrl_CV/imu', Imu],
}

class Message_Manager:
    def __init__(self) -> None:
        self.msgs = deque(maxlen=30)

    def get_msgs(self, msg):
        self.msgs.append(msg)

class SensorListener:
    def __init__(self, sensor_info):
        sub_topic, pub_topic, msg_type = sensor_info
        rospy.Subscriber(sub_topic, msg_type, self.callback)
        self.pub_data = rospy.Publisher(pub_topic, msg_type, queue_size=1)
        self.bridge = CvBridge()
        self.msg_manager = Message_Manager()
        self.sensor_data = None
        self.data_received = False
        
    def callback(self, msg):
        self.msg_manager.get_msgs(msg)

    def gathering_msg(self):
        if len(self.msg_manager.msgs) > 1:
            self.datas = [x.data for x in self.msg_manager.msgs]
            self.times = [x.header.stamp.nsecs for x in self.msg_manager.msgs]
            self.data_received = True
    


class Camera_Image_Listener(SensorListener):
    def __init__(self, sensor_info):
        super().__init__(sensor_info)

    def msg_to_image(self):
        self.data = self.bridge.imgmsg_to_cv2(self.sensor_data, "bgr8")
        self.data_received = True

    def pub_img(self, image):
        data = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.pub_data.publish(data)

    def gathering_msg(self):
        if len(self.msg_manager.msgs) > 1:
            self.datas = [x for x in self.msg_manager.msgs]
            self.times = [x.header.stamp.nsecs for x in self.msg_manager.msgs]
            self.data_received = True

class LiDAR_PointCloud_Listener(SensorListener):
    def __init__(self, sensor_info):
        super().__init__(sensor_info)
        
class GPS_GNSS_Listener(SensorListener):
    def __init__(self, sensor_info):
        super().__init__(sensor_info)

    def gathering_msg(self):
        if len(self.msg_manager.msgs) > 1:
            self.datas = self.msg_manager.msgs
            self.times = [x.header.stamp.nsecs for x in self.msg_manager.msgs]
            self.data_received = True
    
    def pub_msg(self, ):
        if len(self.msg_manager.msgs) >1:
            self.pub_data.publish(self.msg_manager.msgs[-1])


class IMU_Motion_Listener(SensorListener):
    def __init__(self,sensor_info):
        super().__init__(sensor_info)

    def gathering_msg(self):
        if len(self.msg_manager.msgs) > 1:
            self.datas = [[x.orientation.x, x.orientation.y, x.orientation.z, x.orientation.w] for x in self.msg_manager.msgs]
            self.times = [x.header.stamp.nsecs for x in self.msg_manager.msgs]
            self.data_received = True


class SensorConfig:
    def __init__(self, platform_name) -> None:
        if platform_name == 'carla':
            self.sensor_config = CARLA_SENSOR_CONFIG
