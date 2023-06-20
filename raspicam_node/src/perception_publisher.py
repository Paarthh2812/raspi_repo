#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image,CompressedImage
import cv2
from cv_bridge import CvBridge,CvBridgeError
import numpy as np

cap = cv2.VideoCapture('/dev/video1', cv2.CAP_V4L)
print(cap.isOpened())

if not cap.isOpened():
    print("cannot open the camera")
    
bridge = CvBridge()

def main():
    rospy.init_node("image_publisher", anonymous= False)
    pub_comp = rospy.Publisher("/image/compressed",CompressedImage,queue_size=1)
    # pub = rospy.Publisher("/image",Image,queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        print(np.shape(frame))
        # msg2 = bridge.cv2_to_imgmsg(frame,encoding="bgr8")
        # pub.publish(msg2)
        msg = bridge.cv2_to_compressed_imgmsg(frame)
        pub_comp.publish(msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
