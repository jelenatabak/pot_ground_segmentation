#!/usr/bin/env python3

import rospy

import ros_numpy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header 

from geometry_msgs.msg import PointStamped

from segmentation_class_2d import Segmentor
from pc_segmentation_class import PCSegmentor

from dynamic_reconfigure.server import Server
from pot_ground_segmentation.cfg import HsvMaskConfig
from pc_helpers import point_cloud, point_cloud_gray


import cv2 as cv
import numpy as np
import math 
import pcl

import tf2_ros

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import time

def main():
    rospy.init_node("min_y_extraction")
    pc_sub = rospy.Subscriber("/segmented_cloud_glob", PointCloud2, pc_callback)
    rospy.spin()


def pc_callback(pc_message):
    pc_pub_plane = rospy.Publisher("/segmented_plane", PointCloud2, queue_size=10)
    ground_center_pub = rospy.Publisher("/ground_center", PointStamped, queue_size=10)

    segmentation_3d = PCSegmentor(z_lower_limit = 0.3, initial_delta_z = 0.01, z_upper_limit = 0.42)
    segmentation_3d.set_pointcloud(pc_message)
    pc_pc2 = segmentation_3d.segment()

    if pc_pc2 is None:
        print("Blind...")
        return

    points_glob = ros_numpy.point_cloud2.pointcloud2_to_array(pc_pc2)
    

    points = []
    for row in points_glob:
        points.append([row[0], row[1], row[2], row[3]])
    points = np.array(points)

    points_g = points[:,0:3]


    # cloud = pcl.PointCloud()
    # cloud.from_array(np.array(points_g))
    # seg = cloud.make_segmenter_normals(ksearch=50)
    # seg.set_optimize_coefficients(True)
    # seg.set_model_type(pcl.SACMODEL_PLANE)
    # seg.set_normal_distance_weight(0.05)
    # seg.set_method_type(pcl.SAC_RANSAC)
    # seg.set_max_iterations(5)
    # seg.set_distance_threshold(0.5)
    # inliers, model = seg.segment()

    cloud = pcl.PointCloud()
    cloud.from_array(np.array(points_g))
    seg = cloud.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PERPENDICULAR_PLANE)
    seg.set_axis(0., 0., 1.0)
    seg.set_eps_angle(0.2)
    seg.set_normal_distance_weight(0.05)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(200)
    seg.set_distance_threshold(0.01)
    inliers, model = seg.segment()


    if len(inliers)>1:

        points2 = points[inliers, :]
        points2 = np.array(points2)
        pc = point_cloud(points2, "panda_link0")
        pc_pub_plane.publish(pc)

        sums = np.sum(points2[:,:3], 1)
        not_nan = points2[np.logical_and(~np.isnan(sums), ~np.isinf(sums)),:3]
        not_nan = np.array(not_nan)
        not_nan=not_nan.transpose()


        try:

            msg = PointStamped()
            msg.header = Header(frame_id="panda_link0", stamp=rospy.Time.now())
            msg.point.x = np.median(not_nan[0][:])
            msg.point.y = np.median(not_nan[1][:]-0.0) # 8cm od sredine za impedancija eksp s pikanjem
            msg.point.z = np.median(not_nan[2][:])

            print(msg.point.x)
            print(msg.point.y)
            print(msg.point.z)
  
            min_y = np.min(not_nan[1][:])
            print(min_y)
            
            ground_center_pub.publish(msg)
        except IndexError:
            print("ups")
            pass

    # pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(pc_message)
    # y_values = pc_array['y']
    # y_min = np.min(y_values)
    # print(y_min)


if __name__ == "__main__":
    main()