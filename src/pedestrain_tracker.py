#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 3 17:48:42 2019

@author: ag345
"""

import rospy
from pedestrain_tracking.msg import Centroids, Centroids_array
from centroidtracker_with_kalmanfilter import CentroidTracker

def callback(data):
    # get the list of centroids we need

    list_of_centroids =  [(_.x, _.y) for _ in data.data]
    bbs_centers = [_.bb_center for _ in data.data]
    bbs_sizes = [_.bb_size for _ in data.data]
    empty_list = data.empty_or_not# it's a list of boolean values

    # extract time information
    time_step = data.data[0].seconds + data.data[0].nanoseconds * 1e-9
    
    #Use the CentroidTracker class to do the update and predict by the kalmanfitler
    pedestrians_dict, asso_bbs_centers, asso_bbs_sizes, ped_visibility, ped_direction, area_of_ellipse, num_of_matches,\
                    sum_of_distance = ct.update(list_of_centroids, bbs_centers, bbs_sizes, time_step, empty_list)
    #Initialize an array for all pedestrians

    tracked_pedestrians = Centroids_array()
    tracked_pedestrians.num_of_matches = num_of_matches
    tracked_pedestrians.sum_of_distance = sum_of_distance

    for key, value in pedestrians_dict.items():
        #Initialization must be inside the loop, otherwise at each time 
        #p will be overwriten by new value
        p = Centroids() 
        p.id = key
        p.x, p.y = value
        p.bb_center, p.bb_size, p.pedestrian_visible, p.size = asso_bbs_centers[key], asso_bbs_sizes[key], ped_visibility[key], area_of_ellipse[key]
        p.direction.x, p.direction.y = ped_direction[key]
        tracked_pedestrians.data.append(p) 
    
    rospy.loginfo('+'*50)
    
    #Pubilsh the position of all pedestrians as Pedestrians_array
    pub = rospy.Publisher('kalman_output', Centroids_array, queue_size=10)
    pub.publish(tracked_pedestrians)
    #rospy.loginfo(area_of_ellipse)
    rospy.loginfo(tracked_pedestrians.data) 
    #rospy.loginfo(list_of_centroids)

def centroids_listener():
    rospy.init_node('centriods_listener', anonymous = True)
    #Initialize the centroid tracker
    global ct
    ct = CentroidTracker()
    rospy.Subscriber('centroid2', Centroids_array, callback)
    rospy.spin()
    
if __name__ == '__main__':
    centroids_listener()


