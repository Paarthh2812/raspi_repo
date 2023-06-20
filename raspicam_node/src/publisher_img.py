#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image,CompressedImage
import cv2
from cv_bridge import CvBridge,CvBridgeError
import numpy as np

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
print(cap.isOpened())

if not cap.isOpened():
    print("cannot open the camera")
    
bridge = CvBridge()

def main():
    rospy.init_node("image_publisher", anonymous= False)
    pub = rospy.Publisher("/raspicam_node/image/compressed",CompressedImage,queue_size=1)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        fr = np.resize(frame,(640,360))
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
        pub.publish(msg)


if __name__ == '__main__':
        try:
            main()
        except rospy.ROSInterruptException:
            pass
 
