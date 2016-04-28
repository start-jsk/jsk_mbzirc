#/usr/bin/env python

import roslib
roslib.load_manifest("jsk_mbzirc_tasks")

import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from sklearn.neighbors import NearestNeighbors

import numpy as np
from  collections import Counter
import sys
import math
import cv2
import random
import time

sub_image_ = '/downward_cam/camera/image'
#sub_image_ = '/image_publisher/output'

pub_image_ = None
pub_topic_ = '/output'

def plot_image(name, image):
    cv2.namedWindow(str(name), cv2.WINDOW_NORMAL)
    cv2.imshow(str(name), image)
        
#skelonize  use the one in jsk_perception
def skeletonize(img):
    skel = np.zeros(img.shape,np.uint8)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(5,5))
    done = False
    size = np.size(img)
    while(not done):
        eroded = cv2.erode(img,element)
        temp = cv2.dilate(eroded,element)
        temp = cv2.subtract(img,temp)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
 
        zeros = size - cv2.countNonZero(img)
        if zeros==size:
            done = True
    plot_image("skelon", skel)
    return skel

# function
def detect_and_filter_keypoints(im_gray):
    image = cv2.cvtColor(im_gray, cv2.COLOR_GRAY2BGR)
    corners = cv2.goodFeaturesToTrack(im_gray, 400, 0.005, 5, useHarrisDetector=True)
    corners = np.int0(corners)
    
    temp_corners = []
    for i in corners:
        x,y = i.ravel()
        temp_corners.append((x,y))
        cv2.circle(image, (x, y), 3, (0, 0, 255), -1)

    #find the neigbours
    nearest_neigb = NearestNeighbors(n_neighbors = 4, algorithm="kd_tree").fit(np.array(temp_corners))
    distances, indices = nearest_neigb.kneighbors(np.array(temp_corners))
    
    #print distances
    dist_thresh = 10 #pixel
    good_corners = []
    #cache_table = np.zeros((corners.shape[0]), dtype=np.bool)
    
    for i, index in enumerate(indices):  # fix to not repeat on done array
        sum_dist = []
        j = 0
        for idx in index:
            sum_dist.append(np.sum(distances[idx]))
        weight_sum = np.sum(np.abs(np.array(sum_dist) - np.mean(np.array(sum_dist))))
        
        print weight_sum

        if weight_sum < dist_thresh:
            good_corners.append(idx)
    
    for i in good_corners:
        x = corners[i][0][0]
        y = corners[i][0][1]
        cv2.circle(image, (x, y), 3, (0, 255, 0), -1)

    plot_image("lines", im_gray)
    plot_image("edge1", image)

    return

def detect_lines(im_edge, image):
    lines = cv2.HoughLines(im_edge, 1, np.pi/180.0, 100)
    #lines = cv2.HoughLinesP(im_edge, 1,np.pi/180, 100) #, minLineLength = 600, maxLineGap = 100)
    
    im_gray = np.zeros((image.shape[0], image.shape[1], 1), np.uint8)

    # filter based on theta and rho -----todo
    """"
    good_lines = []
    for rho,theta in lines[0]:
        for r, t in lines[0]:
            if math.fabs(rho - r) < 10 and math.fabs(theta - t) < np.pi/8 and rho != r:
                print "SAME: ", rho, ", ", r , "\t" , theta * (180.0/np.pi)
    #---------------------"""""

    for rho,theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        cv2.line(im_gray,(x1, y1),(x2, y2),(255), 3)
        
    # reduce the width to single dim
    plot_image("orig_line", im_gray)
    im_gray = skeletonize(im_gray)
    
    im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detect_and_filter_keypoints(im_gray)

    return

def detect_edges(image):
    
    im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    im_edge = cv2.Canny(im_gray, 100, 200)
    (contours, _) = cv2.findContours(im_edge.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    
    detect_lines(im_edge, image)

    
    #junction_point(contours[22], image)
    #cv2.drawContours(image, contours, 22, (0, 255, 0), 3)
    #plot_image("edge", image)

def image_callback(img_msg):
    bridge = CvBridge()
    cv_img = None
    try:
        cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except Exception as e:
        print (e)

    # timer
    start = time.time()

    #detect_edges(cv_img)
    im_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    detect_and_filter_keypoints(im_gray)

    end = time.time()
    rospy.logwarn("TIME: %s", str(end - start))

    # cv2.namedWindow("image", cv2.WINDOW_NORMAL)
    # cv2.imshow("image", cv_img)
    cv2.waitKey(3)

        
def subscribe():
    rospy.Subscriber(sub_image_, Image, image_callback)
    

def onInit():
    global pub_image_
    pub_image_ = rospy.Publisher(pub_topic_, Image, queue_size=10) 
    subscribe()

def main():
    rospy.init_node('track_region_mapping', anonymous=True)
    onInit()
    rospy.spin()

if __name__ == "__main__":
    main()

