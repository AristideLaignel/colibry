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
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import CameraInfo
import std_msgs
from geometry_msgs.msg import Point32
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import pyransac3d as pyrsc

import sys


class get_image:


    def __init__(self):

        print("Init Get Image Class")

        self.already_init = False

        image_topic = "/camera/color/image_raw"
        image_depth_topic = "/camera/depth/image_aligned_to_color_raw"
        camera_info_topic = "/camera/color/camera_info"

        # Set up your subscriber and define its callback
        rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.Subscriber(image_depth_topic, Image, self.image_depth_callback)
        rospy.Subscriber(camera_info_topic, CameraInfo, self.get_camera_info)

        # Instantiate CvBridge
        self.bridge = CvBridge()

        self.init = False
        self.get_cam_param = False
        self.filter = False
        if sys.argv[1] == "filter" : 
            print("Filter")
            self.filter = True

    def image_callback(self, msg):

        print("Received an image!")

        self.color_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

        try:    

            if self.already_init != True:

                self.already_init = True

                print("NOT initialize")

                color_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                #cv2.imshow('image', depth_image)

                # imgplot = plt.imshow(depth_image)
                #plt.show()

                self.init_image(color_image)
                print("init IS DONE")
                
                
                #h = std_msgs.msgs.Header()
                #h.stamp = rospy.Time.now()


        except CvBridgeError:
            print('erreur')
        # else:
        #     # Save your OpenCV2 image as a jpeg 
        #     cv2.imwrite('camera_depth.jpeg', depth_image)


    def image_depth_callback(self, msg):

        if self.init : 
            print("GEt depth image and initialize")
            # Convert your ROS Image message to OpenCV2 bgr8
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

            self.fill_pointcloud()

    def fill_pointcloud(self):

        point = Point32()

        self.pointcloud_msg = PointCloud()

        # h = std_msgs.msgs.Header()
        self.pointcloud_msg.header.stamp = rospy.Time.now()
        self.pointcloud_msg.header.frame_id = "camera_link"
        # self.pointcloud_msg.points = nb_point
        #self.pointcloud_msg.channels = 3
        

        pix_x_top_left = self.points_clicked[0]
        pix_y_top_left = self.points_clicked[1]

        pix_x_down_right = self.points_clicked[2]
        pix_y_down_right = self.points_clicked[3]

        

        points = []
        for y in range(pix_y_down_right-pix_y_top_left):
                for x in range(pix_x_down_right-pix_x_top_left):

                    # pixel coordinate of the aera clicked by the user
                    u = pix_x_top_left+x
                    v = pix_y_top_left+y
                    
                    # depth for this pixel
                    depth = self.depth_image[v][u]/1000.0
                    #print("Depth : ", depth)

                    if depth != 0:

                        point.x = ((u-self.c_x ) / self.f_x)*depth
                        point.y = ((v-self.c_y ) / self.f_y)*depth
                        point.z = depth

                        print("Depth : ", depth)

                        if self.filter :
                            pt = []
                            pt.append(point.x)
                            pt.append(point.y)
                            pt.append(point.z)
                            points.append(pt)
                        else :
                            self.pointcloud_msg.points.append(point)

                        point = Point32()

        if self.filter :

            array_points = np.array(points)
            plane = pyrsc.Plane()
            best_eq, best_inliers = plane.fit(array_points, 0.01)

            print("Point best: ", best_inliers)

            for index_point in best_inliers:

                point_pc = Point32()
                point_pc.x = array_points[index_point][0]
                point_pc.y = array_points[index_point][1]
                point_pc.z = array_points[index_point][2]
                self.pointcloud_msg.points.append(point_pc)

        print("POintCLoud Lenght : ", self.pointcloud_msg.points.__len__())
        self.pointcloud_pub.publish(self.pointcloud_msg)
        print("Publish Pointcloud")

    def init_image(self, image):
        # displaying the image

        self.count = 0
        self.points_clicked = []

        cv2.imshow('image', image)

        print("Click first on the top left corner of the pointcloud that you want and then on the bottom right corner")
        
        cv2.setMouseCallback('image', self.click_event)
        # wait for a key to be pressed to exit
        cv2.waitKey(0)  
        
        

        # close the window
        cv2.destroyAllWindows()

    def click_event(self, event, x, y, flags, params):
        
        # checking for left mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN and self.count != 2:

            self.count += 1
            # displaying the coordinates
            # on the Shell
            print("Clicked Point : ", x, ' ', y)
            point_clicked_x = x
            point_clicked_y = y

            self.points_clicked.append(point_clicked_x)
            self.points_clicked.append(point_clicked_y)

            if self.count == 2 : 
                cv2.destroyAllWindows()
                print("Get All points ",self.init)
                self.pointcloud_pub = rospy.Publisher('area_pointcloud', PointCloud, queue_size=100000)
                #self.pointcloud_msg = PointCloud()
                #print("fail to inti poinrtcloud")
                self.init = True
                print("fail to inti is truepoinrtcloud")

    def get_camera_info(self, msg):


        if self.get_cam_param == False:

            self.f_x = msg.K[0]
            self.f_y = msg.K[4]

            self.c_x = msg.K[2]
            self.c_y = msg.K[5]

            self.get_cam_param = True

    

    def average_depth(depth_image):

        pix_x_top_left = 161
        pix_y_top_left = 128
        pix_x_down_right = 507
        pix_y_top_right = 321

        print("Depthimage :", depth_image[142][164])
        #cv2.imwrite('camera_color_simple.jpeg', depth_image)
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




def main():

    image_class = get_image()
    rospy.init_node('image_listener')
    try:
        rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down ROS Get Image")
      cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
