#!/usr/bin/env python3

import rospy
from my_robot_tutorial.srv import ImageWithCoordinates
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Callback function to display the received image
def show_image(image):
    cv_bridge = CvBridge()
    cv_image = cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    cv2.imshow("Drone Image", cv_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Function to request an image from the drone
def request_image():
    try:
        rospy.wait_for_service("image_request_service")  # Wait for the service to be available

        # Create a service proxy to communicate with the drone
        image_request_proxy = rospy.ServiceProxy("image_request_service", ImageWithCoordinates)

        # Send request to the drone
        response = image_request_proxy()

        # Display the received image
        if response.success:
            rospy.loginfo("Image received successfully.")
            show_image(response.image)
        else:
            rospy.logwarn("Failed to receive image from the drone.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node("drone_image_client_node", anonymous=True)
    request_image()  # Request an image from the drone
