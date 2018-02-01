# -*- coding: utf-8 -*-
"""
This file is used to accomplish the road tracing based extend kalman filter
Created on Jan 08 2017

!Note: we use the row and col to represent the road coordinate rather than x y,
       the theta is calculated using xOy coordinate. 
@author: Xiaoqiang Liu
"""

#we test git
# calculate the state of start point
from road_kalman import RoadEKF
import numpy as np
import sys
sys.path.append("../code")

def vector_direction(v):
    """ Returns the direction angle of vectors v1::

            >>> vector_direction((1, 0))
            0.0
            >>> angle_between((1, 1)
    """
    
	base = np.array([1, 0])
    v = v / np.linalg.norm(v)
    return np.sign(v[1]) * np.arccos(np.clip(np.dot(v, base), -1.0, 1.0))
 
def get_line(skeleton):
	#the fuction need to return the road
	#using hough transform to detect line in binary_image, the direction can be
	#get form the vector calculation
	 
	lines = cv2.HoughLinesP(sk.astype(np.uint8),rho=1,theta=np.pi/180,threshold=100,
                minLineLength=minLineLength,maxLineGap=maxLineGap)
		
	return lines

def get_profile(image, r, c, theta, road_width=10):
    # we use row and col coordinate
	src = np.array([r - road_width * np.cos(theta), c + road_width * np.sin(theta)])
    dst = np.array([r + road_width * np.cos(theta), c - road_width * np.sin(theta)])
    profile = skimage.measure.profile_line(image, src, dst, linewidth=3, order=0, mode='constant', cval=0.0)
    return profile
	
def match_profile(profile, Set_profile):
	# using least square get the shift between the two profile
	corrcoef = numpy.corrcoef(profile, Set_Profile)
	measuredData = np.array(yvalues['int1'])
	calibrationData = np.array(yvalues['int0'])

	A = np.vstack( [measuredData, np.ones(len(measuredData))]).T
	gain,offset = np.linalg.lstsq(A, calibrationData)[0]
	return corrcoef, gain, offset
	
def main():
    
	road = []
    dt = 3
    road_tracker = RoadEKF(dt)
    road_tracker.P = np.array(4*4)    # uncertainty covariance
    road_tracker.Q = np.array(4*4)    # process uncertainty
    road_tracker.R = np.array(4*4)    # state uncertainty

    for line in lines:
	    subroad = []
		for x1,y1,x2,y2 in line:
			vector12 = np.array([x1-x2, y1-y2])
			subroad.append([x1, y1])
			road_tracker.x = np.array([[x1, y1, vector_direction(vector12), 0]])
			profile = get_profile(image, x1, y1, vector_direction(vector12))
			i = 0
			while(True):
			    road_tracker.predict()
				predict_x = road_tracker.x
				z, profile = get_measurment(predict_x, profile) #the function has not implement
				road_tracker.update(z)
				subroad.append([road_tracker.x[0,0], road_tracker.x[1,0]])
			
			
			vector21 = np.array([x2-x1, y2-y1])
			subroad.append([x2, y2])
			road_tracker.x = np.array([[x2, y2, vector_direction(vector21), 0]])
			
			profile = get_profile(image, x2, y2, vector_direction(vector21))
			i = 0
			while(True):
			    road_tracker.predict()
				predict_x = road_tracker.x
				z, profile = get_measurment(predict_x, profile) #the function has not implement
				road_tracker.update(z)
				subroad.insert(0, [road_tracker.x[0,0], road_tracker.x[1,0]])
		road.append(subroad)
    
	
if __name__ == '__main__':
    main()