#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 3 17:38:11 2019

@author: ag345
"""
from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
import math
import scipy.linalg as linalg
from scipy.optimize import linear_sum_assignment

class CentroidTracker():
    def __init__(self, maxDisappeared=5):
        # initialize the next unique pedestrian ID along with two ordered
        # dictionaries used to keep track of mapping a given pedestrian
        # ID to its centroid and number of consecutive frames it has
        # been marked as "disappeared", respectively
        self.nextPedestrianID = 0
        self.estimates = OrderedDict() # OrderedDict will remeber the order in which its contents are added
        self.disappeared = OrderedDict()
        self.maxDisappeared = maxDisappeared
        self.kfs = OrderedDict() # list of kalman filters
        self.last_time_step = 3.01
        self.bbs_centers = OrderedDict()
        self.bbs_sizes = OrderedDict()
        self.visibility = OrderedDict()
        self.direction = OrderedDict()
        self.area_of_ellipse = OrderedDict()
        self.sum_of_distance = 0.
        self.num_of_matches = 0
        
    def cal_size_of_ellipse(self, P):
        '''
        P : nd.array shape (4,4)
        covariance matrix

        '''
        #firstly, takes only the uncertainty of x and y, turn P into 2x2 matrix
        P = np.array([[P[0,0],P[2,0]],
                      [P[0,2],P[2,2]]])
        U, s, _ = linalg.svd(P)
        width = math.sqrt(s[0])
        height = math.sqrt(s[1])

        area = 1./4 * math.pi * width * height
        return area

    def register(self, measurement, bb_center, bb_size, dt):
        # when registering an pedestrian we use the next available pedestrian
        # ID to store the centroid
        self.estimates[self.nextPedestrianID] = measurement
        # set kalman filter for each new pedestrian and initialize the position with measurement
        # self.kfs[self.nextPedestrianID] = self.kf(np.array([[measurement[0], -0.267023, measurement[1], -0.318225]]).T, dt)
        self.kfs[self.nextPedestrianID] = self.kf(np.array([[measurement[0], 0, measurement[1], 0]]).T, dt)
        self.area_of_ellipse[self.nextPedestrianID] = self.cal_size_of_ellipse(self.kfs[self.nextPedestrianID].P)
        self.bbs_centers[self.nextPedestrianID] = bb_center
        self.bbs_sizes[self.nextPedestrianID] = bb_size
        self.disappeared[self.nextPedestrianID] = 0 #when registration, initialize the number of disappeared times of this new coming pedestrian as 0
        self.visibility[self.nextPedestrianID] = True
        self.direction[self.nextPedestrianID] = [0, 0]
        self.nextPedestrianID += 1
        
    def deregister(self, pedestrianID):
        # to deregister an pedestrian ID we delete the pedestrian ID from
        # both of our respective dictionaries
        del self.estimates[pedestrianID]
        del self.disappeared[pedestrianID]
        del self.kfs[pedestrianID]
        del self.bbs_centers[pedestrianID]
        del self.bbs_sizes[pedestrianID]
        del self.visibility[pedestrianID]
        del self.direction[pedestrianID]
        del self.area_of_ellipse[pedestrianID]
        
    def kf(self,x, dt):
        '''
            dt is time difference betweeen two frames
        '''
        kf = KalmanFilter(dim_x=4, dim_z=2)
        kf.F = np.array([[1, dt, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, dt],
                        [0, 0, 0, 1]])
        kf.u = 0.
        kf.H = np.array([[1, 0, 0, 0], # for initialization of R we should take the measurement errors for position
                        [0, 0, 1, 0]])

        kf.R = np.diag([0.01, 0.01]) # for initialization of R we should take the measurement errors for position
        vel_max = 10# maximum velocity of a human
        q = Q_discrete_white_noise(dim=2, dt=dt, var=0.01) # here var is changable
        
        kf.Q = block_diag(q, q)
        kf.x = x
        variance_of_vel = np.diag([vel_max**2, vel_max**2])
        kf.P = block_diag(kf.R[0,0], variance_of_vel[0,0], kf.R[1,1], variance_of_vel[1,1]) #initialize P with R and square of maximum velocity
        return kf

    def update(self, measurements, bbs_centers, bbs_sizes, time_step, empty_list):
        # calculate the time difference between two neighboured frames
        dt = time_step - self.last_time_step
        self.last_time_step = time_step
        # check to see if the list of input bounding box centroids
        # is empty.
        
        if empty_list == 1:
            # loop over any existing tracked pedestrians and mark them
            # as disappeared
            for pedestrianID in list(self.disappeared.keys()):
                self.disappeared[pedestrianID] += 1
                self.visibility[pedestrianID] = False

                # if we have reached a maximum number of consecutive
                # frames where a given pedestrian has been marked as
                # missing, deregister it
                if self.disappeared[pedestrianID] > self.maxDisappeared:
                    self.deregister(pedestrianID)
            # update the position of not disappeared pedestrians by prediction
            try:
                for pedestrianID in list(self.estimates.key()):
                    self.kfs[pedestrianID].predict()
                    self.estimates[pedestrianID] = [self.kfs[pedestrianID].x[0, 0], self.kfs[pedestrianID].x[2, 0]]
                    self.direction[pedestrianID] = [self.kfs[pedestrianID].x[1, 0] * dt, self.kfs[pedestrianID].x[3, 0] * dt]
                    self.area_of_ellipse[pedestrianID] = self.cal_size_of_ellipse(self.kfs[pedestrianID].P)
            except AttributeError:
                print('no measurements!!')
            # terminate early as there are no centroids or tracking info to update
            return self.estimates, self.bbs_centers, self.bbs_sizes, self.visibility, self.direction, self.area_of_ellipse,\
                    self.sum_of_distance, self.num_of_matches 
        # initialize an array of input centroids for the current frame
        newMeasurements = np.asarray(measurements)
        
        # if we are currently not tracking any pedestrians take the input
        # centroids and register each of them
        if len(self.estimates) == 0:
            for i in range(0, len(newMeasurements)):
                self.register(newMeasurements[i], bbs_centers[i], bbs_sizes[i], dt)

        # otherwise, we are currently tracking pedestrians so we need to
        # try to match the measurements to existing pedestrians
        # centroids
        else:
            # grab the set of pedestrian IDs and corresponding centroids
            pedestrianIDs = list(self.estimates.keys())
            # predict next position firstly, then do the association part
            for pedestrianID in pedestrianIDs:
                self.kfs[pedestrianID].predict()
                self.estimates[pedestrianID] = [self.kfs[pedestrianID].x[0, 0], self.kfs[pedestrianID].x[2, 0]]
            # get the predicted position of pedestrians
            pedestrianCentroids = list(self.estimates.values())
            
            D = dist.cdist(np.array(pedestrianCentroids), newMeasurements)
            rows, cols = linear_sum_assignment(D)

            '''
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]
            '''
            usedRows = set() # Unordered collections of unique elements
            usedCols = set()
            for (row, col) in zip(rows, cols):
                # if the distance between associated ids a and b is no larger than the threshold, 
                # the id a will be deregistered and b will be registered
                if D[row,col] > 4:
                    self.register(newMeasurements[col], bbs_centers[col], bbs_sizes[col], dt)
                    pedestrianID = pedestrianIDs[row]
                    self.visibility[pedestrianID] = False
                    self.disappeared[pedestrianID] += 1
                    # deregister the pedestrian if he has dsiappeared long enough
                    if self.disappeared[pedestrianID] > self.maxDisappeared:
                        self.deregister(pedestrianID)
                else:
                    pedestrianID = pedestrianIDs[row]
                    self.kfs[pedestrianID].update(newMeasurements[col])
                    self.estimates[pedestrianID] = [self.kfs[pedestrianID].x[0, 0], self.kfs[pedestrianID].x[2, 0]]
                    self.area_of_ellipse[pedestrianID] = self.cal_size_of_ellipse(self.kfs[pedestrianID].P)
                    self.direction[pedestrianID] = [self.kfs[pedestrianID].x[1, 0] * dt, self.kfs[pedestrianID].x[3, 0] * dt]
                    self.bbs_centers[pedestrianID] = bbs_centers[col]
                    self.bbs_sizes[pedestrianID] = bbs_sizes[col]
                    self.disappeared[pedestrianID] = 0
                    self.visibility[pedestrianID] = True
                    self.num_of_matches += 1
                    self.sum_of_distance += D[row, col]

                usedRows.add(row)
                usedCols.add(col)

            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)
            
            #if D.shape[0] >= D.shape[1]:164
                # loop over the unused row indexes
            for row in unusedRows:
                    # grab the pedestrian ID for the corresponding row
                    # index and increment the disappeared counter164
                pedestrianID = pedestrianIDs[row]
                self.visibility[pedestrianID] = False
                self.disappeared[pedestrianID] += 1
                # check to see if the number of consecutive
                # frames the pedestrian has been marked "disappeared"
                # for warrants deregistering the pedestrian
                if self.disappeared[pedestrianID] > self.maxDisappeared:
                    self.deregister(pedestrianID)
                
            for col in unusedCols:
                self.register(newMeasurements[col], bbs_centers[col], bbs_sizes[col], dt)

            # otherwise, if the number of input centroids is greater
            # than the number of existing pedestrian centroids we need to
            # register each new input centroid as a trackable pedestrian
            '''
            else:
                for col in unusedCols:
                    self.register(newMeasurements[col], bbs_centers[col], bbs_sizes[col], dt)
            '''
        # return the set of trackable pedestrians
        # return self.pedestrians
        return self.estimates, self.bbs_centers, self.bbs_sizes, self.visibility, self.direction, self.area_of_ellipse,\
                self.num_of_matches, self.sum_of_distance
