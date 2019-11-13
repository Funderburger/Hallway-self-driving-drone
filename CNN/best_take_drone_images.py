import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
from keras.models import load_model
from keras.preprocessing.image import load_img
from keras.preprocessing.image import img_to_array
import tensorflow as tf
from tensorflow.python.keras.backend import set_session
import numpy as np
import os
from std_msgs.msg import Empty 
from geometry_msgs.msg import Twist  
import tensorflow as tf

sess = tf.Session()
graph = tf.get_default_graph()

# IMPORTANT: models have to be loaded AFTER SETTING THE SESSION for keras! 
# Otherwise, their weights will be unavailable in the threads after the session there has been set

set_session(sess)
model = load_model('third_save.h5')
# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
	cv2_img=cv2.resize(cv2_img,(224,224))
    X=cv2_img.reshape(1,224,224,3)
    global sess
    global graph
  
    with graph.as_default():
        set_session(sess)
        Y=model.predict(X)
        angularVelocity=Y
        print(angularVelocity)
        pubCommand = rospy.Publisher('/cmd_vel',Twist)
        global command
        command = Twist()
        command.linear.x = 0.05
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.z = angularVelocity/2
        pubCommand.publish(command)
        cv2.waitKey(1)

def main():
    rospy.init_node('image_listener')
    image_topic = "/ardrone/front/image_rect_color"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
