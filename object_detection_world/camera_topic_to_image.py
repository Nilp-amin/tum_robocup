#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_folder = "saved_images"
        self.image_count = 0

        # Create the folder if it doesn't exist
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)

        # Subscribe to the image topic
        rospy.Subscriber("/your/image/topic", Image, self.image_callback)

    def image_callback(self, msg):
        try:
            # Convert the ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Save the image to the folder
            image_filename = f"image_{self.image_count}.png"
            image_path = os.path.join(self.image_folder, image_filename)
            cv2.imwrite(image_path, cv_image)

            rospy.loginfo(f"Image saved: {image_path}")

            self.image_count += 1
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

def main():
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
