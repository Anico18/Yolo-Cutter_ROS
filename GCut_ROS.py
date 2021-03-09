import sys
#sys.path.append('/home/alejandro/yolov5-master-interpreter')
import cv2
from colorthief import ColorThief
import numpy as np
import os
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
import rospy
# from darknet_ros_msgs.msg import BoundingBox
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
# Callbacks


def callback(ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    bounding_boxes = [[332.0, 144.0, 409.0, 411.0]]
    for box in bounding_boxes:
        x, y, x1, y1 = box
        roi = img[int(y):int(y1), int(x):int(x1)]
    cv2.imshow('img',roi)
    cv2.waitKey(50)

    # Prepares pub
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', roi)[1]).tostring()
    # Publish new image
    image_pub.publish(msg)

# def callback2(ros_data1):
#    print(ros_data1)

rospy.init_node('Pepper_subscriber')
image_pub = rospy.Publisher("Pepper_Publisher", CompressedImage, queue_size=5)
# Subscriber
rospy.Subscriber('/pepper/camera/front/image_raw/compressed', CompressedImage, callback, queue_size=5)
# rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback2, queue_size=5)
rospy.spin()
