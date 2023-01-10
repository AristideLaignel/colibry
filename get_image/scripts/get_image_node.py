#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import numpy as np

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2 bgr8
        depth_image = bridge.imgmsg_to_cv2(msg, 'passthrough')

        # depth_np_array = np.frombuffer(msg.data, np.uint16)
        # print("Image type :", float(msg.data[142][164]))
        #cv_image_depth = cv2.imdecode(depth_np_array, cv2.IMREAD_UNCHANGED)

        pix_x_top_left = 161
        pix_y_top_left = 128
        pix_x_down_right = 507
        pix_y_top_right = 321

        print("Depthimage :", depth_image[142][164])
        cv2.imwrite('camera_depth3.jpeg', depth_image)
        count = 0 
        average_depth = 0
        for y in range(pix_y_top_right-pix_y_top_left):
            for x in range(pix_x_down_right-pix_x_top_left):
                u = pix_x_top_left+x
                v = pix_y_top_left+y
                depth = depth_image[v][u]
                if depth != 0:
                    count += 1 
                    average_depth += depth
        
        average_depth = average_depth/(count*1000)

        print("Average depth of the object detected are : ", average_depth)
        

        #rospy.on_shutdown(myhook)
    except CvBridgeError:
        print('erreur')
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_depth.jpeg', depth_image)



    # 164, 142
    # 403, 272

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/depth/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
