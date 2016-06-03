#!/usr/bin/env python

import roslib
roslib.load_manifest("jsk_mbzirc_tasks")

import rospy
from cv_bridge import CvBridge
from sklearn.neighbors import NearestNeighbors, KDTree, DistanceMetric
from sensor_msgs.msg import Image, PointCloud2
from jsk_mbzirc_msgs.msg import ProjectionMatrix
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
sub_matrix_ = '/projection_matrix'

pub_image_ = None
pub_topic_ = '/track_region_mapping/output/track_mask'

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

def line_intersection_points(x1, y1, x2, y2, x3, y3, x4, y4):
    denominator = (x1 - x2) * (y3 - y4) - (y1 -y2) * (x3 - x4)
    if denominator == 0:
        return (-1, -1)

    intersect_x = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / denominator
    intersect_y = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / denominator
    
    return (intersect_x, intersect_y)

def detect_lines(im_edge, image):
    lines = cv2.HoughLines(im_edge, 1, np.pi/180, 100)
    
    flag_bit = np.zeros((lines[0].shape[0], 1)) # mark processed lines

    filtered_lines = []
    filter_dist_thresh = 10
    filter_angl_thresh = 10  * (np.pi/180.0) #degrees
    for flag, (rho,theta) in zip(flag_bit, lines[0]):
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 10000*(-b))
        y1 = int(y0 + 10000*(a))
        x2 = int(x0 - 10000*(-b))
        y2 = int(y0 - 10000*(a))
        #cv2.line(image,(x1,y1),(x2,y2),(0,255,0), 2)
        
        if not flag:            
            current_best = []
            for index, (rho0, theta0) in enumerate(lines[0]):
                a0 = np.cos(theta0)
                b0 = np.sin(theta0)
                x00 = a0*rho0
                y00 = b0*rho0
                x10 = int(x00 + 10000*(-b0))
                y10 = int(y00 + 10000*(a0))
                x20 = int(x00 - 10000*(-b0))
                y20 = int(y00 - 10000*(a0))

                line_dist = distance.euclidean((x0, y0), (x00, y00))
                angle_dist = math.fabs(theta - theta0)
                if line_dist < filter_dist_thresh and angle_dist < filter_angl_thresh:
                    #current_best.append((x00, y00, x20, y20))
                    filtered_lines.append((x10, y10, x20, y20))
                    #flag_bit[index] = True

                #print "SAVING: ", line_dist, ",  ", rho, " ", rho0, "\t", theta, " ", theta0
                #plot_image("lines", image)
                #cv2.waitKey(0)
                #print current_best
            #filtered_lines.append(current_best[0]) 
            #filtered_lines.append(current_best[1]) 

    print filtered_lines
    inter_points = []
    for line in filtered_lines:
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        cv2.line(image,(x1,y1),(x2,y2),(0,0, 255), 1)

        for l in filtered_lines:
            x10 = line[0]
            y10 = line[1]
            x20 = line[2]
            y20 = line[3]
            (ptx, pty) = line_intersection_points(x10, y10, x20, y20, x1, y1, x2, y2)
            #print ptx, "\t", pty
            if ptx > 0 and ptx < image.shape[1] and pty > 0 and pty < image.shape[0]:
                inter_points.append([[int(ptx), int(pty)]])
                cv2.circle(image, (int(ptx), int(pty)), 3, (255,0,255), -1)
                
    plot_image("lines", image)
    cv2.waitKey(3)
    return np.array(inter_points)

def vector_angle(vector1, vector2):
    cos_angl = np.dot(vector1, vector2)
    sin_angl = np.linalg.norm(np.cross(vector1, vector2))
    return np.arctan2(sin_angl, cos_angl)

def detect_and_filter_keypoints(im_gray, corner_type='HARRIS'):
    
    image = cv2.cvtColor(im_gray, cv2.COLOR_GRAY2BGR)
    image = cv2.GaussianBlur(image, (5, 5), 0)
    corners = None
    if corner_type is 'HARRIS':
        corners = cv2.goodFeaturesToTrack(im_gray, 400, 0.005, 5, useHarrisDetector=True)
        if corners is None:
            return
        corners = np.int0(corners)
    elif corner_type is 'HOUGH_LINES':
        im_edge = cv2.Canny(im_gray, 100, 200)
        corners = detect_lines(im_edge, im_gray)

    temp_corners = []
    ground_z = 0.0
    for i in corners:
        x,y = i.ravel()
        
        a00 = x * proj_matrix_[2, 0] - proj_matrix_[0, 0]
        a01 = x * proj_matrix_[2, 1] - proj_matrix_[0, 1]
        a10 = y * proj_matrix_[2, 0] - proj_matrix_[1, 0]
        a11 = y * proj_matrix_[2, 1] - proj_matrix_[1, 1]
        bv0 = proj_matrix_[0, 2] * ground_z + proj_matrix_[0, 3] -  \
              x * proj_matrix_[2, 2] * ground_z - x * proj_matrix_[2, 3]
        bv1 = proj_matrix_[1, 2] * ground_z + proj_matrix_[1, 3] -  \
              y * proj_matrix_[2, 2] * ground_z - y * proj_matrix_[2, 3]
        partition = a11 * a00 - a01 * a10
        pos_x = (a11 * bv0 - a01 * bv1) / partition
        pos_y = (a00 * bv1 - a10 * bv0) / partition
        
        temp_corners.append((pos_x, pos_y, ground_z))
        #temp_corners.append((x,y))
        cv2.circle(image, (x, y), 3, (0, 0, 255), -1)
        

    #find the neigbours
    radius_thresh = 6.0
    #n_neighbors = 4,
    nearest_neigb = NearestNeighbors(radius = radius_thresh, \
                                     algorithm="kd_tree", \
                                     leaf_size = 30, \
                                     metric = 'euclidean', \
                                     n_jobs = 1).fit(np.array(temp_corners))
    #distances, indices = nearest_neigb.kneighbors(np.array(temp_corners))
    distances, indices = nearest_neigb.radius_neighbors(np.array(temp_corners))    
    
    

    temp_im = image.copy()
    possible_candidate = []
    for distance, index in zip(distances, indices):  # fix to not repeat on done array        
        centeroid_index = -1
        for dist, idx in zip(distance, index):
            if dist == 0:
                centeroid_index = idx
                break
                
        #print "\033[34m CENTROID: \033[0m", centeroid_index
        #cv2.circle(image, (corners[centeroid_index][0][0],\
        #                   corners[centeroid_index][0][1]), 5, (0, 255, 0), 2)
        #print "INDEX: ", index, "\t", np.average(distance), "\t", distance

        #avg_dist = np.average(distance)
        #if (avg_dist > 2.5 and avg_dist < 5):
        for dist, idx in zip(distance, index):
            angle = vector_angle(temp_corners[centeroid_index], temp_corners[idx]) * (180.0/np.pi)
            if (dist > (radius_thresh/2) and dist < radius_thresh) and(angle > 75 and angle < 180):
                possible_candidate.append(idx)                

        #     print "ANGLE: ", angle, "\t", dist, "\t", idx
        #     cv2.circle(image, (corners[idx][0][0], corners[idx][0][1]), 3, (255, 255, 0), -1)
        #     plot_image("edge1", image)
        #     cv2.waitKey(0)

        # image = temp_im.copy()


    print (possible_candidate)
    
    centroid_point = []
    if len(possible_candidate) > 3:
        aver_x = 0
        aver_y = 0
        for pc in possible_candidate:
            cv2.circle(image, (corners[pc][0][0], corners[pc][0][1]), 4, (255, 255, 0), 1)
            aver_x += corners[pc][0][0]
            aver_y += corners[pc][0][1]
        aver_x /= len(possible_candidate)
        aver_y /= len(possible_candidate)
        centroid_point.append((aver_x, aver_y))
        
        cv2.circle(image, (aver_x, aver_y), 7, (0, 255, 0), -1)
    plot_image("edge1", image)
    
    return (np.array(possible_candidate), np.array(centroid_point))


def detect_edges(image):

    im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    im_edge = cv2.Canny(im_gray, 100, 200)
    #(contours, _) = cv2.findContours(im_edge.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    detect_lines(im_edge, image)    


# side is square region
def vehicle_track_region(image, center_corners, centroid_point, size = 9):
    if center_corners.size < 4 or centroid_point.shape[0] == 0:
        rospy.logerr("NOT ENOUGHT POINTS TO GET REGION")
        return
    #image = cv2.resize(image, (image.shape[1]/2, image.shape[0]/2))
    im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    im_edge = cv2.Canny(im_gray, 100, 200)
    im_mask = cv2.copyMakeBorder(im_edge, 1, 1, 1, 1, cv2.BORDER_REPLICATE)
    
    # region growing
    im_diff = im_mask.copy()
    im_flooded = image.copy()
    (retval, rect) = cv2.floodFill(im_flooded, im_mask, \
                                   (centroid_point[0, 0], centroid_point[0, 1]), (255, 255, 0), \
                                   flags = 8 | (255 << 8))
    im_diff = im_mask - im_diff
    plot_image("diff", im_diff)
    plot_image("contour_pos", im_flooded)

    return im_diff

def image_callback(img_msg):
    if not is_proj_mat_:
        rospy.logwarn("PROJECTION MATRIX NODE ERROR")

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
    corner_points, centroid_point = detect_and_filter_keypoints(im_gray, 'HARRIS')
    
    im_track_mask = vehicle_track_region(cv_img, corner_points, centroid_point)
    if not im_track_mask is None:
        ros_img = bridge.cv2_to_imgmsg(im_track_mask, "mono8")
        ros_img.header = img_msg.header
        pub_image_.publish(ros_img)
    else:
        pass

    end = time.time()
    rospy.logwarn("TIME: %s", str(end - start))
    cv2.waitKey(3)

def projection_matrix_callback(data):
    global proj_matrix_
    proj_matrix_ = np.reshape(data.data, (3, 4))
    global is_proj_mat_
    is_proj_mat_ = True
        
def subscribe():
    #rospy.Subscriber(sub_matrix_, VectorArray, projection_matrix_callback)
    rospy.Subscriber(sub_matrix_, ProjectionMatrix, projection_matrix_callback)
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

