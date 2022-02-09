#!/usr/bin/env python3

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2

class PCSegmentor():
    def __init__(self, pc_msg = None, z_lower_limit = 0, initial_delta_z = 0.07, z_upper_limit = 0.14, y_upper_limit = 1, x_lower_limit = -0.25, x_upper_limit = 0.25):
        self.z_lower_limit = z_lower_limit
        self.z_upper_limit = z_upper_limit
        self.initial_delta_z = initial_delta_z
        self.y_upper_limit = y_upper_limit
        self.x_lower_limit = x_lower_limit
        self.x_upper_limit = x_upper_limit

        self.pc_colored_pub = rospy.Publisher("/colored_pc", PointCloud2, queue_size=10)

        if pc_msg:
            self.set_pointcloud(pc_msg)



    def set_pointcloud(self, pc_msg):
        self.pc_msg = pc_msg
        self.frame_id = pc_msg.header.frame_id

        # 1D np array
        self.points = ros_numpy.point_cloud2.pointcloud2_to_array(self.pc_msg)
        self.points = self.points.flatten()

        # initial filtering
        x_values = self.points['x']
        y_values = self.points['y']
        z_values = self.points['z']

        mask = (y_values < self.y_upper_limit) * ~np.isnan(y_values) * (z_values > self.z_lower_limit) * (z_values < self.z_upper_limit) * (x_values > self.x_lower_limit) * (x_values < self.x_upper_limit)
        self.points_filtered = self.points[mask]

        # sort points based on z value
        self.points_sorted_orig = self.points_filtered[np.argsort(self.points_filtered['z'])]
        self.z_values_orig = self.points_sorted_orig['z']



    def segment(self, publish_colored_pc = True):
        """ Segments the ground from the pointcloud.
        Args:
            publish_colored_pc: if True, publishes pc with colored segments on /colored_pc topic
        Returns:
            filtered sensor_msgs/PointCloud2 message
        """

        delta_z = self.initial_delta_z
        z_lower_limit = self.z_lower_limit
        z_upper_limit = self.z_upper_limit
        z_min = self.z_lower_limit
        z_max = z_min + delta_z

        z_values_curr = self.z_values_orig
        points_sorted = np.copy(self.points_sorted_orig)

        while delta_z > 0.01:
            print("===== NEW ITERATION =====")
            # print("Delta z:" + str(delta_z) + "\n")
            # print("Min z:" + str(z_min))

            point_counter = 0
            sector_counter = 1
            rgba_current = 0.01
            sector_pts = []

            i = 0
            while i < len(z_values_curr):
                z_curr = z_values_curr[i]

                if z_curr < z_max:
                    points_sorted['rgb'][i] = rgba_current
                    point_counter += 1
                    i += 1
                else:
                    sector_pts.append(point_counter)
                    #print("Segment " + str(sector_counter) + ": " + str(point_counter) + " points\n")
                    z_min = z_max
                    #print("Min z:" + str(z_min))
                    z_max = z_min + delta_z

                    point_counter = 0
                    sector_counter += 1
                    rgba_current += 0.02

            sector_pts.append(point_counter)
            #print("Segment " + str(sector_counter) + ": " + str(point_counter) + " points\n")

            index = self.select_segment(sector_pts)

            if index is None:
                return None

            if(publish_colored_pc):
                pc_colored = ros_numpy.point_cloud2.array_to_pointcloud2(points_sorted, frame_id=self.frame_id)
                self.pc_colored_pub.publish(pc_colored)

            # set new limits
            z_lower_limit = z_lower_limit + index*delta_z - delta_z/2
            z_min = z_lower_limit
            z_upper_limit = z_min + 2*delta_z

            delta_z = 0.75*delta_z
            z_max = z_min + delta_z

            # keep only the points inside the new limits
            mask = (self.z_values_orig > z_lower_limit) * (self.z_values_orig < z_upper_limit)
            points_sorted = self.points_sorted_orig[mask]
            z_values_curr = points_sorted['z']


        pc = ros_numpy.point_cloud2.array_to_pointcloud2(points_sorted, frame_id=self.pc_msg.header.frame_id)
        return pc



    def select_segment(self, sector_pts):
        """ Select the segmemt based on the number of points and the difference between the current segment and its neighbours.
        Args:
            sector_pts: vector of number of points per segment
        Returns:
            index of selected segment
        """
        delta_sectors = []
        if len(sector_pts) > 1:
            for i in range(len(sector_pts)):
                if i == 0:
                    delta_curr = abs(sector_pts[0]-sector_pts[1])/(sector_pts[0]+sector_pts[1]+1) * sector_pts[0]

                elif i == len(sector_pts)-1:
                    delta_curr = abs(sector_pts[-2]-sector_pts[-1])/(sector_pts[-2]+sector_pts[-1]+1) * sector_pts[-1]

                else:
                    prev = sector_pts[i-1]
                    curr = sector_pts[i]
                    next_ = sector_pts[i+1]

                    d_prev = abs(prev-curr)/(prev+curr+1) * curr
                    d_next_ = abs(curr-next_)/(curr+next_+1) * curr

                    delta_curr = (d_prev+d_next_)/2

                delta_sectors.append(delta_curr)
        else:
            return None

        #print("Scores of segments: " + str(delta_sectors))
        max_value = max(delta_sectors)
        index = delta_sectors.index(max_value)
        #print("Selecting segment: " + str(index+1))

        return index
