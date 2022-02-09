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


class GroundSegment():
    def __init__(self):
        self.bridge = CvBridge()


        self.segmentation = Segmentor()
        self.segmentation_3d = PCSegmentor(z_lower_limit = 0, initial_delta_z = 0.01, z_upper_limit = 0.14)

        self.target_frame = 'world'
        self.source_frame = 'camera'

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        time.sleep(2.)


        self.image_pub = rospy.Publisher("/segmented", Image, queue_size=10)
        self.mask_pub = rospy.Publisher("/segmentation_mask", Image, queue_size=10)
        self.contours_pub = rospy.Publisher("/segmentation_contours", Image, queue_size=10)

        self.image_sub = rospy.Subscriber("/color/image_raw", Image, self.img_callback)

        self.pc_sub = rospy.Subscriber("/depth_registered/points", PointCloud2, self.pc_callback)

        self.pc_pub_loc = rospy.Publisher("/segmented_cloud_loc", PointCloud2, queue_size=10)
        self.pc_pub_glob = rospy.Publisher("/segmented_cloud_glob", PointCloud2, queue_size=10)
        self.pc_pub_plane = rospy.Publisher("/segmented_plane", PointCloud2, queue_size=10)

        self.ground_center_pub = rospy.Publisher("/ground_center", PointStamped, queue_size=10)

        self.srv = Server(HsvMaskConfig, self.dynamic_recon_cb)

        self.pc_array = None    

        self.cloud_loc = None
        self.pc_glob = rospy.Publisher("/full_cloud_glob", PointCloud2, queue_size=10)

    def dynamic_recon_cb(self, config, level):
        mask_lower = (config['hue_L'], config['sat_L'], config['val_L'])
        mask_upper = (config['hue_H'], config['sat_H'], config['val_H'])

        self.segmentation.min_contour_size = config['contour_size']
        self.segmentation.mask_lower = mask_lower
        self.segmentation.mask_upper = mask_upper
        self.segmentation.kernel = config['kernel_size']
        return config

    def img_callback(self, image_message):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_message, 'rgb8')
        except CvBridgeError as e:
            print(e)
            return 
        
        self.segmentation.new_image(cv_image)



    def pc_callback(self, pc_message):

        self.pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(pc_message)
        self.cloud_loc = pc_message
        # self.segmentation.set_pointcloud(pc_message)



def main():

    rospy.init_node("ground_segmentation")


    gs = GroundSegment()


    while not rospy.is_shutdown():

        if gs.segmentation.image is not None:
            masked, mask, contours = gs.segmentation.segment()
            gs.image_pub.publish(gs.bridge.cv2_to_imgmsg(masked, 'rgb8'))
            gs.mask_pub.publish(gs.bridge.cv2_to_imgmsg(mask, 'rgb8'))
            gs.contours_pub.publish(gs.bridge.cv2_to_imgmsg(contours, 'rgb8'))

            if gs.pc_array is not None:
                points = []
                temp_array = gs.pc_array
                gs.pc_array = None
                
                gray_mask_leaves = cv.cvtColor(mask, cv.COLOR_BGR2GRAY)
                gray_mask = cv.bitwise_not(gray_mask_leaves, gray_mask_leaves)



                xmax, ymax = np.shape(gray_mask)
                for ix in range(xmax):
                    for iy in range(ymax):
                        if gray_mask[ix,iy] :
                            (x,y,z,rgb) = temp_array[ix, iy]
                            if not math.isnan(x):
                                points.append((x,y,z,rgb))

                if len(points)>1:
                    m, _ = np.shape(points)
                    points = np.array(points)

                    pc_rgb = point_cloud(points, gs.source_frame)
                    gs.pc_pub_loc.publish(pc_rgb)
                    
                    try:
                        trans = gs.tfBuffer.lookup_transform(gs.target_frame, gs.source_frame, gs.cloud_loc.header.stamp)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        print("Did not get transform")
                        return

                    cloud_out_rgb = do_transform_cloud(pc_rgb, trans)
                    gs.pc_pub_glob.publish(cloud_out_rgb)
                  
                    # cloud_glob = do_transform_cloud(gs.cloud_loc, trans)
                    # gs.pc_glob.publish(cloud_glob)

                    gs.segmentation_3d.set_pointcloud(cloud_out_rgb)
                    pc_pc2 = gs.segmentation_3d.segment()



                    if pc_pc2 is None:
                        print("Blind...")
                        continue

                    # print(pc_pc2.fields)


                    points_glob = ros_numpy.point_cloud2.pointcloud2_to_array(pc_pc2)
                    # points_g = []
                    # for row in points_glob:
                    #     points_g.append([row[0], row[1], row[2]])
                    

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
                    # seg.set_distance_threshold(0.01)
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

                    # print("model") # ax + by + cz + d = 0 
                    # print(model) 

                    # points = []
                    # for row in points_glob:
                    #     points.append([row[0], row[1], row[2], row[3]])
                    # points = np.array(points)


                    if len(inliers)>1:

                        points2 = points[inliers, :]
                        points2 = np.array(points2)
                        pc = point_cloud(points2, gs.target_frame)
                        gs.pc_pub_plane.publish(pc)

                        sums = np.sum(points2[:,:3], 1)
                        # bla = points2[~np.isnan(sums) and ~np.isinf(sums), :3]

                        not_nan = points2[np.logical_and(~np.isnan(sums), ~np.isinf(sums)),:3]


                        # not_nan = []
                        # for p in points2:
                        #     if not math.isnan(np.sum(p)) and not math.isinf(np.sum(p)):
                        #         not_nan.append(p[:3])
                        not_nan = np.array(not_nan)
                        not_nan=not_nan.transpose()

                        # print(np.shape(not_nan))

                        try:

                            msg = PointStamped()
                            msg.header = Header(frame_id=gs.target_frame, stamp=rospy.Time.now())
                            msg.point.x = np.median(not_nan[0][:])
                            msg.point.y = np.median(not_nan[1][:]-0.0) # 8cm od sredine za impedancija eksp s pikanjem
                            msg.point.z = np.median(not_nan[2][:])
                            #msg.point.z = np.median(not_nan[2][:]+0.165) # pomak od panda_link8 do vrha senzora
                            print(msg.point.x)
                            print(msg.point.y)
                            print(msg.point.z)

                            msg.point.y = np.min(not_nan[1][:]) 
                            print(msg.point.y)

                            gs.ground_center_pub.publish(msg)
                        except IndexError:
                            print("ups")
                            pass


    rospy.spin()


if __name__ == "__main__":
    main()