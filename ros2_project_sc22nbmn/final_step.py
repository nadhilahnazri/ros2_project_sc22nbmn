# The task
# The robot should be able to move and detect the RGB coloured boxed (all three boxes).
# When a blue box is detected, the robot should move up to the blue box and stop at roughly
# within a radius of 1 meter from the centre of the blue box.

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base
        # Initialise any flags that signal a colour has been detected (default to false)
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        self.sensitivity = 10
        self.red_found = False
        self.green_found = False
        self.blue_found = False

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10) #10Hz
        self.too_close = False

        
    def callback(self, data):
        
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)
        
        # Set the upper and lower bounds for the two colours you wish to identify
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

        hsv_red_lower1 = np.array([0, 100, 100])
        hsv_red_upper1 = np.array([self.sensitivity, 255, 255])
        hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
        hsv_red_upper2 = np.array([180 + self.sensitivity, 255, 255])

        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but particular colours using the cv2.inRange() method
        # Do this for each colour
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
        red_mask1 = cv2.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1)
        red_mask2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        # mask_image = cv2.bitwise_or(green_image,red_image)
        # #green_mask = cv2.bitwise_and(image,image,mask = green_image)
        # #both_images = cv2.bitwise_or(green_mask,red_mask)
        
        # final_image = cv2.bitwise_and(image,image,mask = mask_image)
        
        
        # To combine the masks you should use the cv2.bitwise_or() method
        # You can only bitwise_or two images at once, so multiple calls are necessary for more than two colours
        combined_mask = cv2.bitwise_or(green_mask, cv2.bitwise_or(blue_mask, red_mask))

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to...
        #bitwise and an image with itself and pass the mask to the mask parameter (rgb_image,rgb_image, mask=mask)
        # As opposed to performing a bitwise_and on the mask and the image.
        green_image = cv2.bitwise_and(image, image, mask=green_mask)
        blue_image = cv2.bitwise_and(image, image, mask=blue_mask)
        red_image = cv2.bitwise_and(image, image, mask=red_mask)
        combined_image = cv2.bitwise_and(image, image, mask=combined_mask)

        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.imshow('Original Feed', image)
        cv2.imshow('Green Objects', green_image)
        cv2.imshow('Blue Objects', blue_image)
        cv2.imshow('Red Objects', red_image)
        cv2.imshow('Combined Detection', combined_image)

        cv2.waitKey(3)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()

    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass
    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
    main()