#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 3 17:38:11 2019

@author: ag345
"""
from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np

class CentroidTracker():
	def __init__(self, maxDisappeared=20):
		# initialize the next unique pedestrian ID along with two ordered
		# dictionaries used to keep track of mapping a given pedestrian
		# ID to its centroid and number of consecutive frames it has
		# been marked as "disappeared", respectively
		self.nextPedestrianID = 0
		self.pedestrians = OrderedDict() # OrderedDict will remeber the order in which its contents are added
		self.pedestriansWithAssociation = OrderedDict() # dict{ID:[x_t-1, y_t-1, x_t, y_t]}
		self.disappeared = OrderedDict()

		# store the number of maximum consecutive frames a given
		# pedestrian is allowed to be marked as "disappeared" until we
		# need to deregister the pedestrian from tracking
		self.maxDisappeared = maxDisappeared

	def register(self, centroid):
		# when registering an pedestrian we use the next available pedestrian
		# ID to store the centroid
		self.pedestrians[self.nextPedestrianID] = centroid
		self.pedestriansWithAssociation[self.nextPedestrianID] = np.array([0, 0, centroid[0], centroid[1]])
		self.disappeared[self.nextPedestrianID] = 0
		self.nextPedestrianID += 1

	def deregister(self, pedestrianID):
		# to deregister an pedestrian ID we delete the pedestrian ID from
		# both of our respective dictionaries
		del self.pedestrians[pedestrianID]
		del self.disappeared[pedestrianID]
		del self.pedestriansWithAssociation[pedestrianID]

	def update(self, centroids):
		# check to see if the list of input bounding box centroids
		# is empty. 
		if len(centroids) == 0:
			# loop over any existing tracked pedestrians and mark them
			# as disappeared
			for pedestrianID in list(self.disappeared.keys()):
				self.disappeared[pedestrianID] += 1

				# if we have reached a maximum number of consecutive
				# frames where a given pedestrian has been marked as
				# missing, deregister it
				if self.disappeared[pedestrianID] > self.maxDisappeared:
					self.deregister(pedestrianID)

			# return early as there are no centroids or tracking info
			# to update
			#return self.pedestrians
			return self.pedestriansWithAssociation

		# initialize an array of input centroids for the current frame
		inputCentroids = np.asarray(centroids)

		# if we are currently not tracking any pedestrians take the input
		# centroids and register each of them
		if len(self.pedestrians) == 0:
			for i in range(0, len(inputCentroids)):
				self.register(inputCentroids[i])

		# otherwise, we are currently tracking pedestrians so we need to
		# try to match the input centroids to existing pedestrian
		# centroids
		else:
			# grab the set of pedestrian IDs and corresponding centroids
			pedestrianIDs = list(self.pedestrians.keys())
			pedestrianCentroids = list(self.pedestrians.values())

			# compute the distance between each pair of pedestrian
			# centroids and input centroids, respectively -- our
			# goal will be to match an input centroid to an existing
			# pedestrian centroid
			D = dist.cdist(np.array(pedestrianCentroids), inputCentroids)

			# in order to perform this matching we must (1) find the
			# smallest value in each row and then (2) sort the row
			# indexes based on their minimum values so that the row
			# with the smallest value as at the *front* of the index
			# list
			rows = D.min(axis=1).argsort()

			# next, we perform a similar process on the columns by
			# finding the smallest value in each column and then
			# sorting using the previously computed row index list
			cols = D.argmin(axis=1)[rows]

			# in order to determine if we need to update, register,
			# or deregister an pedestrian we need to keep track of which
			# of the rows and column indexes we have already examined
			usedRows = set()
			usedCols = set()

			# loop over the combination of the (row, column) index
			# tuples
			for (row, col) in zip(rows, cols):
				# if we have already examined either the row or
				# column value before, ignore it
                
				if row in usedRows or col in usedCols:
					continue

				# otherwise, grab the pedestrian ID for the current row,
				# set its new centroid, and reset the disappeared
				# counter
				pedestrianID = pedestrianIDs[row]
				self.pedestriansWithAssociation[pedestrianID] = np.array([self.pedestrians[pedestrianID][0], self.pedestrians[pedestrianID][1], 
																		inputCentroids[col][0], inputCentroids[col][1]])
				self.pedestrians[pedestrianID] = inputCentroids[col]
				self.disappeared[pedestrianID] = 0

				# indicate that we have examined each of the row and
				# column indexes, respectively
				usedRows.add(row)
				usedCols.add(col)

			# compute both the row and column index we have NOT yet
			# examined
			unusedRows = set(range(0, D.shape[0])).difference(usedRows)
			unusedCols = set(range(0, D.shape[1])).difference(usedCols)

			# in the event that the number of pedestrian centroids is
			# equal or greater than the number of input centroids
			# we need to check and see if some of these pedestrians have
			# potentially disappeared
			if D.shape[0] >= D.shape[1]:
				# loop over the unused row indexes
				for row in unusedRows:
					# grab the pedestrian ID for the corresponding row
					# index and increment the disappeared counter
					pedestrianID = pedestrianIDs[row]
					self.disappeared[pedestrianID] += 1
					self.pedestriansWithAssociation[pedestrianID] = np.array([self.pedestrians[pedestrianID][0], self.pedestrians[pedestrianID][1], 
																			self.pedestrians[pedestrianID][0], self.pedestrians[pedestrianID][1]])

					# check to see if the number of consecutive
					# frames the pedestrian has been marked "disappeared"
					# for warrants deregistering the pedestrian
					if self.disappeared[pedestrianID] > self.maxDisappeared:
						self.deregister(pedestrianID)

			# otherwise, if the number of input centroids is greater
			# than the number of existing pedestrian centroids we need to
			# register each new input centroid as a trackable pedestrian
			else:
				for col in unusedCols:
					self.register(inputCentroids[col])

		# return the set of trackable pedestrians
		# return self.pedestrians
		return self.pedestriansWithAssociation
		# return the centroids in this form --> dict{ID:[x_t-1, y_t-1, x_t, y_t]}
