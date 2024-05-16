#!/usr/bin/env python3

import rospy
from my_robot_tutorial.srv import ImageWithCoordinates, ImageWithCoordinatesResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import random

class SurveillanceDrone:
    def __init__(self):
        rospy.init_node("surveillance_drone")

        # Advertise the image request service
        rospy.Service("image_request_service", ImageWithCoordinates, self.capture_image)

        # Initialize CvBridge
        self.cv_bridge = CvBridge()

        # Path to the directory containing images
        self.image_dir = "/home/harry/catkin_ws/src/my_robot_tutorial/scripts/imgs/"

        # List of available images
        self.image_files = os.listdir(self.image_dir)

        # Start the surveillance loop
        self.start_surveillance()

    def start_surveillance(self):
        rospy.loginfo("Surveillance drone started.")
        while not rospy.is_shutdown():
            # Simulate random anomaly detection
            anomaly_detected = random.choice([True, False])
            if anomaly_detected:
                # Capture and send an image with coordinates
                response = self.detect_anomaly()
                rospy.loginfo("Anomaly detected at coordinates: (%f, %f)", response.latitude, response.longitude)

            # Wait for a random time before the next detection
            detection_interval = random.randint(5, 45)
            rospy.sleep(detection_interval)

    def detect_anomaly(self):
        # Select a random image
        image_filename = random.choice(self.image_files)
        image_path = os.path.join(self.image_dir, image_filename)

        # Read the image
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            rospy.logerr("Failed to read the image: %s", image_path)
            return

        # Convert the image to ROS image message
        image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        # Generate random coordinates
        latitude = random.uniform(11.2503, 11.5503)
        longitude = random.uniform(13.4167, 13.9167)

        # Create and return the response
        return ImageWithCoordinatesResponse(success=True, image=image_msg, latitude=latitude, longitude=longitude)

    def capture_image(self, req):
        """
        Capture an image from the drone's camera and return it as a response.
        """
        # In a real implementation, capture image from the drone's camera
        # For now, we'll simulate image capture by returning a random image from the directory
        return self.detect_anomaly()

if __name__ == "__main__":
    try:
        drone = SurveillanceDrone()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
