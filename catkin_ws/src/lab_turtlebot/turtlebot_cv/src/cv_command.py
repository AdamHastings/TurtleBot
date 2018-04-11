#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist

class CVControl:

    def __init__(self):

        # Turtlebot command publisher
        self.cmd_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        self.cmd = Twist()

        # Image subscriber
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/decompressed_img", Image, self.img_callback)
        self.old_gray = None
        self.IMG_HEIGHT = 480
        self.IMG_WIDTH = 640
        self.OF_WINDOW_UPPER_BOUND = 2 * (self.IMG_HEIGHT / 4)
        self.OF_WINDOW_LOWER_BOUND = 3 * (self.IMG_HEIGHT / 4)
        self.OF_WINDOW_WIDTH = (self.IMG_WIDTH / 3)
        self.GRID_SIZE = 8

    def get_w(self, cv_image):
        if (self.old_gray is None):
            self.old_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            return 0

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        opticalFlow = cv2.calcOpticalFlowFarneback(self.old_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        self.old_gray = gray

        opticalFlow = opticalFlow.T
        # print(opticalFlow)
        opticalFlow_x = opticalFlow[0]
     

        ofx_l = np.array([])
        ofx_r = np.array([])

        for j in range(self.OF_WINDOW_UPPER_BOUND, self.OF_WINDOW_LOWER_BOUND, self.GRID_SIZE):
            for i in range(0, self.OF_WINDOW_WIDTH, self.GRID_SIZE):
                ofx_l = np.append(ofx_l, opticalFlow_x[i,j])
            for i in range((2*self.OF_WINDOW_WIDTH), self.IMG_WIDTH, self.GRID_SIZE):
                ofx_r = np.append(ofx_r, opticalFlow_x[i,j])

        # print("ofx_r: " + str(ofx_r))
        # print("ofx_l: " + str(ofx_l))

        sum_ofxl = np.sum(ofx_l)
        sum_ofxr = np.sum(ofx_r)

        # print("sum_ofxr: " + str(sum_ofxr) + "     sum_ofxl: " + str(sum_ofxl))
        sum_abs_ofxl = np.sum(np.absolute(ofx_l))
        sum_abs_ofxr = np.sum(np.absolute(ofx_r))

        k = .3 # Hand-picked scalar
        w = k * ((sum_ofxl - sum_ofxr)/(sum_abs_ofxl + sum_abs_ofxr))
        print("w = " + str(w))
        return (np.sign(w) / 2)

    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # Draw circle on image
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # Send Velocity command to turtlebot
        v = 0.2
        w = self.get_w(cv_image)
        v = 0
        w = 0.0  # positive w means ccw spin
        self.send_command(v, w)

    def send_command(self, v, w):
        # Put v, w commands into Twist message
        self.cmd.linear.x = v
        self.cmd.angular.z = w

        # Publish Twist command
        self.cmd_pub.publish(self.cmd)

def main():
    print("Canyon Following")
    ctrl = CVControl()
    rospy.init_node('image_converter')
    try:
        print("finished init")
        rospy.spin()
        print("finished spin")
    except KeyboardInterrupt:
        print "Shutting down"
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
