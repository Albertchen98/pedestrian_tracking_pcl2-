#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 14:30:52 2020

@author: ag345
"""

import rospy
import pandas as pd
from pedestrain_tracking.msg import Centroids_array

def callback(data):
    ellipse_list = [(_.id,_.size) for _ in data.data]
    num_of_matches.append(data.num_of_matches)
    sum_of_distance.append(data.sum_of_distance)

    for i in ellipse_list:
        key = i[0]
        if key in dict_of_ellipses:
            dict_of_ellipses[key].append(i[1])
        else:
            dict_of_ellipses.update({key: [i[1]]})
    
    #rospy.loginfo(dict_of_ellipses)
    
    
    
    
def evaluation():
    global dict_of_ellipses
    dict_of_ellipses = {}
    global num_of_matches
    num_of_matches = []
    global sum_of_distance
    sum_of_distance = []
    
    rospy.init_node('evaluation', anonymous = True)
    rospy.Subscriber('kalman_output', Centroids_array, callback)
    rospy.spin()
    
    df = pd.DataFrame(dict([ (k,pd.Series(v)) for k,v in dict_of_ellipses.items()]))
    df.to_csv('/fzi/ids/ag345/test.csv',index=False)
    MOPT = sum(sum_of_distance)/sum(num_of_matches)
    print('MOPT is:', MOPT)
    
    
if __name__ == '__main__':
    dict_of_ellipses = evaluation()
    
    