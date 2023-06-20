#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image,CompressedImage
import cv2
from cv_bridge import CvBridge,CvBridgeError
import numpy as np

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L) # RIght
cap2 = cv2.VideoCapture('/dev/video2',cv2.CAP_V4L) # Left
print(cap.isOpened())
print(cap2.isOpened())

if not cap.isOpened():
    print("cannot open the camera")
if not cap2.isOpened():
    print("cannot open the camera")
    
bridge = CvBridge()

def main():
    rospy.init_node("image_publisher", anonymous= False)
    pubR = rospy.Publisher("/imageR/compressed",CompressedImage,queue_size=1)
    pubL = rospy.Publisher("/imageL/compressed",CompressedImage,queue_size=1)
    # pub = rospy.Publisher("/image",Image,queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frameR = cap.read()
        ret2,frameL = cap2.read()
        # msg2 = bridge.cv2_to_imgmsg(frame,encoding="bgr8")
        # pub.publish(msg2)
        print(np.shape(frameR))
        print(np.shape(frameL))
        msg = bridge.cv2_to_compressed_imgmsg(frameR)
        msgL = bridge.cv2_to_compressed_imgmsg(frameL)
        pubR.publish(msg)
        pubL.publish(msgL)
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
