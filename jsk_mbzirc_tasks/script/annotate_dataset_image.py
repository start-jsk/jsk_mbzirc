
#!/usr/bin/evn python

import numpy as np
import cv2
import sys
import rospy
import os

from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Polygon, Point
from cv_bridge import CvBridge

# ros message topics
sub_image_ = '/image_publisher/output'
sub_polygon_ = sub_image_+ '/screenrectangle'

# read or write infos
directory_ = '/home/krishneel/Desktop/mbzirc/data/'
data_type_ = 'negative'

# increment when cropping
window_stride_ = 40

is_rectangle_ = None
rect_ = None

def help():
    print "\033[32m "
    print "----------------------------------------------------------------------------"
    print "\t\tNODE FOR TEAM JSK MBZIRC @ U-TOKYO"
    print "----------------------------------------------------------------------------"
    print "| Node to crop full resolution image into patches for training"
    print "| Use jsk_perception image_publisher to publish the image to crop into dataset"
    print "| Use image_view2 to mark a rectangular region which is sliding window size"
    print "| Provide the directory to save the cropped patches and .txt file"
    print "| Provide window stride as incrementor"
    print "| Provide the label and type of dataset (Positive or negative)"
    print "----------------------------------------------------------------------------"
    print "\033[0m"

def rectangle_callback(msg):
    global is_rectangle_ 
    global rect_
    rect = []
    if is_rectangle_ is None:
        rect.append(msg.polygon.points[0].x)
        rect.append(msg.polygon.points[0].y)
        width = msg.polygon.points[1].x - msg.polygon.points[0].x
        height = msg.polygon.points[1].y - msg.polygon.points[0].y
        rect.append(width)
        rect.append(height)
        rect_ = np.array(rect)
        is_rectangle_ = True
    else:
        rospy.logwarn("THIS CLICK IS NOT REGISTERED.")

def annotate_image(image, wsize):
    if image is None:
        rospy.logfatal("CANNOT ANNOTATE EMPTY IMAGE")
        return
    if data_type_ is None or directory_ is None:
        rospy.logfatal("SPECIFY DATA TYPE and DIRECTORY")
        return

    if not os.path.exists(directory_ + data_type_):
        os.makedirs(directory_ + data_type_)

    out_file = open(directory_ + data_type_ + '.txt', 'w')
    icount = 0
    for j in xrange(0, image.shape[0] - window_stride_, window_stride_):
        for i in xrange(0, image.shape[1] - window_stride_, window_stride_):
            roi = image[j:j+wsize[1], i:i+wsize[0]]
            write_path = directory_ + data_type_ + '/'+ str(icount) + '.jpg' 
            cv2.imwrite(write_path, roi)
            out_file.write(write_path + ' ' + str(-1) + '\n')
            icount += 1
    out_file.close()

def image_callback(img_msg):
    if not is_rectangle_:
        rospy.logerr("MARK A RECT REGION FOR WINDOW SIZE")
        return
    bridge = CvBridge()
    cv_img = None
    try:
        cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except Exception as e:
        print (e)
    
    wsize = (int(rect_[2]), int(rect_[3]))
    
    annotate_image(cv_img, wsize)

    cv2.imshow("image", cv_img)
    cv2.waitKey(3)

def subscribe():
    rospy.Subscriber(sub_polygon_, PolygonStamped, rectangle_callback)
    rospy.Subscriber(sub_image_, Image, image_callback)


def onInit():
    subscribe()

def main():
    help()
    rospy.init_node('annotate_dataset_image', anonymous=True)
    onInit()
    rospy.spin()

if __name__ == "__main__":
    main()
    
