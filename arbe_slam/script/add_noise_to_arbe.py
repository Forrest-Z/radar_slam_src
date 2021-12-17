#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rosbag
import sys
from sensor_msgs import point_cloud2
import numpy as np
import time

motor_angle = -1
motor_angle_prev = -1
motor_angle_offset = 74
motor_angle_range = 3 / 180. * 3.14159
data_list = []
power_list = []
time_duration = 0


class PointCloudData(object):
    def __init__(self, radar_data):
        self.x = radar_data[0]
        self.y = radar_data[1]
        self.z = radar_data[2]
        self.range = radar_data[4]
        self.azimuth = radar_data[5]
        self.elevation = radar_data[6]
        self.doppler = radar_data[7]
        self.power = radar_data[8]

        self.r_bin = radar_data[9]
        self.varphi_bin = radar_data[10]
        self.theta_bin = radar_data[11]
        self.vr_bin = radar_data[12]
        self.rcs_bin = radar_data[13]

        self.timestamp = radar_data[14]


def update_motor_angle(motor_angle_msg):
    global motor_angle
    motor_angle = (motor_angle_msg.data - motor_angle_offset) / 180. * 3.14159
    # print(motor_angle)


def calculate_power(pointcloud_msg):
    global time_prev
    time_prev = time.perf_counter()
    global power_list, motor_angle, motor_angle_prev
    if motor_angle != motor_angle_prev:
        if power_list:
            data_list.append([motor_angle, len(power_list), np.mean(power_list), np.std(power_list, ddof=1)])
            print(motor_angle / 3.14159 * 180)
            print("find : ", data_list[-1])
            power_list = []
            motor_angle_prev = motor_angle

    # pointcloud.data = b''

    radar_data = point_cloud2.read_points(pointcloud_msg, skip_nans=True)
    for data in radar_data:
        point = PointCloudData(data)
        if point.range > 17 and point.range < 18 \
                and point.azimuth < motor_angle + motor_angle_range \
                and point.azimuth > motor_angle - motor_angle_range \
                and point.z > 0.5 and point.z < 2.3:
            power_list.append(point.power)
            print("\tget : ", point.range, point.azimuth, point.z, point.power)

if __name__ == "__main__":
    file_name = "arbe_calibration.npy"
    if len(sys.argv) > 1:
        file_name = sys.argv[1]
    
    bag_file = "/home/qinguoyu/andrew_ws/bag/arbe/fw_v143/calibration/arbe_calibration_01.bag"

    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=['/arbe/rviz/pointcloud']):
        if topic == '/arbe/rviz/pointcloud':
            print('/arbe/rviz/pointcloud', msg.width)
            calculate_power(msg)
    bag.close()

    np.save(file_name, np.array(data_list))