#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 16 17:38:11 2019

@author: oh372
"""

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from pedestrain_tracking.msg import Centroids_array
from geometry_msgs.msg import Point

    
def callback(data):
    
    pedestrians = data.data
    
    #initialize an array for all boundingboxes
    bb_array = MarkerArray()
    
    #delete all previous markers when new Point Cloud is incoming by introducing the first marker of the marker_array only for the deletion purpose and not connected to any points belonging to a pedestrian
    marker = Marker()
    marker.header.frame_id = "velo_link"
    marker.type = Marker.CUBE
    #DELETE ALL MARKERS IN RVIZ: marker.action = Marker.DELETEALL
    marker.action = Marker.DELETEALL
    bb_array.markers.append(marker)
    
    if len(pedestrians) > 0:
        for element in range(len(pedestrians)):
            #initialize boundingbox as marker
            marker = Marker()
            marker.ns = "bounding box"
            marker.header.frame_id = "velo_link"
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            
            #to surround the Pedestrians visually in RVIZ, you need some offset around the data points
            bb_offset = 0.1
            #set bounding box size (pedestrians width, length, height)
            marker.scale.x = pedestrians[element].bb_size.x + 2*bb_offset
            marker.scale.y = pedestrians[element].bb_size.y + 2*bb_offset
            marker.scale.z = pedestrians[element].bb_size.z + 2*bb_offset


            
            # check if pedestrian is visible
            if pedestrians[element].pedestrian_visible == True:              
                #set middle point of bounding box
                marker.pose.position.x = pedestrians[element].bb_center.x
                marker.pose.position.y = pedestrians[element].bb_center.y
                marker.pose.position.z = pedestrians[element].bb_center.z
                
                #set bounding box color: (1,1,1)=white
                marker.color.r=1
                marker.color.g=1
                marker.color.b=1
                marker.color.a = 0.5 #a is necessary for bounding box to be visible
            
            #if pdestrian is occluded/disappeared, display a different bounding box
            elif pedestrians[element].pedestrian_visible == False:
                #set middle point of bounding box
                marker.pose.position.x = pedestrians[element].x
                marker.pose.position.y = pedestrians[element].y
                marker.pose.position.z = pedestrians[element].bb_center.z
                
                #set bounding box color: (1,1,1)=white
                marker.color.r=0.5
                marker.color.g=0.1
                marker.color.b=0.2
                marker.color.a = 0.5 #a is necessary for bounding box to be visible
            
            #additional Text-Marker showing number of pedestrian's cluster
            cluster = Marker()
            cluster.ns = "cluster"
            cluster.header.frame_id = "velo_link"
            cluster.type = Marker.TEXT_VIEW_FACING
            cluster.pose.position.x = marker.pose.position.x
            cluster.pose.position.y = marker.pose.position.y
            cluster.pose.position.z = marker.pose.position.z + 0.5*pedestrians[element].bb_size.z + 2*bb_offset
            cluster.action = Marker.ADD
            cluster.scale.z = 0.3
            cluster.text = str(pedestrians[element].id)
            cluster.color.r = 50
            cluster.color.g = 10
            cluster.color.b = 10
            cluster.color.a = 1
            
            #additional marker showing the predicted movement of the pedestrian
            arrow = Marker()
            arrow.ns = "arrow"
            arrow.header.frame_id = "velo_link"
            arrow.type = Marker.ARROW
            arrow.scale.x = 0.01
            arrow.scale.y = 0.1
            arrow.scale.z = 0.1
            point = Point()
            point.x = marker.pose.position.x
            point.y = marker.pose.position.y
            point.z = marker.pose.position.z - 0.5*pedestrians[element].bb_size.z
            arrow.points.append(point)
            point2 = Point()
            point2.x = point.x + pedestrians[element].direction.x*-150
            point2.y = point.y + pedestrians[element].direction.y*-150
            rospy.loginfo(point2.x)
            rospy.loginfo(point2.y)
            point2.z = point.z
            arrow.points.append(point2)
            arrow.action = Marker.ADD
            arrow.color.r = 50
            arrow.color.g = 10
            arrow.color.b = 10
            arrow.color.a = 1
            
            #append markers to marker array
            bb_array.markers.append(marker)
            bb_array.markers.append(cluster)
            bb_array.markers.append(arrow)
            
            # Renumber the marker IDs
            id = 0
            for m in bb_array.markers:
                m.id = id
                id += 1

        rospy.loginfo(bb_array)

        #Publish bounding boxes
        pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        pub.publish(bb_array)
    

def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('kalman_output', Centroids_array, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()