#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 16 17:38:11 2019

@author: oh372
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from pedestrain_tracking.msg import Centroids
from pedestrain_tracking.msg import Centroids_array


def callback(data):
    
    #save timestamp of point cloud
    secs = data.header.stamp.secs
    nanosecs = data.header.stamp.nsecs
    
    #make a list of all points of incoming PointCloud2.msg but only relevant fields x,y,cluster,label  
    cloud_points = list(point_cloud2.read_points(cloud=data, field_names = ('x','y','z','cluster','label')))
    
    dictionary={}
    
    for element in cloud_points:
        data_point = element
        # only proceed with points that have the label "pedestrian", 
        if data_point[4] == 3:
            rospy.loginfo(data_point)
            key = data_point[3]

            #check if pedestrian already exists in dictionary by its cluster
            if key in dictionary:
                # append array of the key with new data points if key already exists in dictionary
                dictionary[key] = np.vstack([dictionary[key], [data_point[0], data_point[1], data_point[2]]])
                
            else:
                #make new entry in dictionary for new key
                dictionary.update({key: np.array([data_point[0], data_point[1], data_point[2]])})

    centroids_array = Centroids_array()
    
    if len(dictionary) == 0:
        centroid_info = Centroids()
        centroids_array.data.append(centroid_info)
        centroids_array.empty_or_not = True
        centroid_info.seconds = secs
        centroid_info.nanoseconds = nanosecs
    
    else:
        centroids_array.empty_or_not = False
        for cluster in dictionary:
            centroid_info = Centroids()
            centroid_info.id = int(cluster)
            if dictionary[cluster].size > 3:
                #calculate centroid for KF
                centroid = np.median(dictionary[cluster], axis=0)
                
                #delete points from cluster which are far away and are supposed to be clustered wrong
                index = 0
                idx = []
                for point in dictionary[cluster]:
                    if abs(point[0]-centroid[0]) > 2 or abs(point[1]-centroid[1]) > 2 or abs(point[2]-centroid[2]) > 2:
                        idx.append(index)
                    index+=1
                
                #if the clustering appears to be totally wrong (several pedestrians clustered together), skip this cluster
                if len(idx)==dictionary[cluster].size/3:
                    continue
                    
                else:
                    dictionary[cluster] = np.delete(dictionary[cluster], idx, axis=0)                     
                    # calculate Bounding Box parameters center and size of bounding box for vision_msgs/BoundingBox3D Message, Z-COORDINATE IS MISSING!
                    # maximum_coord = array ([x(max), y(max)])
                    rospy.loginfo(dictionary[cluster])
                    maximum_coord = np.amax(dictionary[cluster], axis=0)
                    minimum_coord = np.amin(dictionary[cluster], axis=0)
                    pose_center = np.array([float((maximum_coord[0]+minimum_coord[0])/2), float((maximum_coord[1]+minimum_coord[1])/2), float((maximum_coord[2]+minimum_coord[2])/2)])
                                
            else:
                centroid = dictionary[cluster]
                maximum_coord = dictionary[cluster]
                minimum_coord = dictionary[cluster]
                pose_center = dictionary[cluster]
                
            
            centroid_info.pedestrian_visible = True
            centroid_info.x = centroid[0]
            centroid_info.y = centroid[1]
            centroid_info.bb_center.x = pose_center[0]
            centroid_info.bb_center.y = pose_center[1]
            centroid_info.bb_center.z = pose_center[2]
            centroid_info.bb_size.x = maximum_coord[0] - minimum_coord[0]
            centroid_info.bb_size.y = maximum_coord[1] - minimum_coord[1]
            centroid_info.bb_size.z = maximum_coord[2] - minimum_coord[2]
            centroid_info.seconds = secs
            centroid_info.nanoseconds = nanosecs
    
            centroids_array.data.append(centroid_info)    
    
    #rospy.loginfo(dictionary)
    #rospy.loginfo(centroids_array.data)

    #Publish centroids as Floatlist.msg
    pub = rospy.Publisher('centroid2', Centroids_array, queue_size=10)
    pub.publish(centroids_array)




def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/cocar/ibeo_lux_201/cloud', PointCloud2, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()