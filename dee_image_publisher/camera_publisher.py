#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

cap = cv2.VideoCapture(0)
print("Camera opened: ", cap.isOpened())
bridge = CvBridge()

def main(args=None):
    rclpy.init(args=args)

    node = Node('camera_publisher')
    pub = node.create_publisher(Image, 'webcam', 10)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("Error opening camera")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(msg)

    cap.release()
    cv2.destroyAllWindows()

    rclpy.shutdown()

if __name__ == '__main__':
    main()