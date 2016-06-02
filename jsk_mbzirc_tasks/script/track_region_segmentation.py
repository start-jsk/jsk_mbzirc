#/usr/bin/env python

import roslib
roslib.load_manifest("jsk_mbzirc_tasks")

import rospy
from cv_bridge import CvBridge
from sklearn.neighbors import NearestNeighbors, KDTree, DistanceMetric
from sensor_msgs.msg import Image, PointCloud2
from jsk_recognition_msgs.msg import VectorArray
from std_msgs.msg import Header
from scipy.spatial import distance

import numpy as np
from  collections import Counter
import sys
import math
import cv2
import random
import time

sub_image_ = '/downward_cam/camera/image'
#sub_image_ = '/image_publisher/output'
sub_matrix_ = '/paramatrix'

pub_image_ = None
pub_topic_ = '/track_region_segmentation/output/track_mask'

proj_matrix_ = None
is_proj_mat_ = False

def help():
    print "\033[32m "
    print "----------------------------------------------------------------------------"
    print "\t\tNODE FOR TEAM JSK MBZIRC @ U-TOKYO"
    print "----------------------------------------------------------------------------"
    print "| ROS node for detecting the x-region on the vehicle track."
    print "| Using projection mapping the pixel distance are converted to world coordinate"    
    print "| system and the known prior is used for filtering."
    print "| The final track region is segmented using region growing with edge and color."
    print "----------------------------------------------------------------------------"
    print "\033[0m"

def plot_image(name, image):
    cv2.namedWindow(str(name), cv2.WINDOW_NORMAL)
    cv2.imshow(str(name), image)



def detect_edge_contours(image):
    im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    im_gray = cv2.GaussianBlur(im_gray, (7, 7), 0)
    im_edge = cv2.Canny(im_gray, 30, 100)
    (contours, _) = cv2.findContours(im_edge.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        color = np.random.randint(0,255,(3)).tolist()
        cv2.drawContours(image,[cnt],0,color,2)
    plot_image("contours", image)

    

def image_callback(img_msg):
    # if not is_proj_mat_:
    #     rospy.logwarn("PROJECTION MATRIX NODE ERROR")

    bridge = CvBridge()
    cv_img = None
    try:
        cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except Exception as e:
        print (e)

    # timer
    start = time.time()
    detect_edge_contours(cv_img)
    
    im_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

    end = time.time()
    rospy.logwarn("TIME: %s", str(end - start))
    cv2.waitKey(3)

def projection_matrix_callback(data):
    global proj_matrix_
    proj_matrix_ = np.reshape(data.data, (3, 4))
    global is_proj_mat_
    is_proj_mat_ = True
        
def subscribe():
    rospy.Subscriber(sub_matrix_, VectorArray, projection_matrix_callback)
    rospy.Subscriber(sub_image_, Image, image_callback)

def onInit():
    global pub_image_
    pub_image_ = rospy.Publisher(pub_topic_, Image, queue_size=10) 
    subscribe()

def main():
    rospy.init_node('track_region_segmentation', anonymous=True)
    onInit()
    rospy.spin()

if __name__ == "__main__":
    main()

