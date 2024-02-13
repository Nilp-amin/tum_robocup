#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver:
    '''
    Image saver class that receives the topic name of the robots video feed and the nth image it is supposed to save as input.
    '''
    def __init__(self, topic_name, save_every_nth_image):
        self.bridge = CvBridge()
        self.image_folder = "saved_images"
        self.image_count = 0
        self.topic_name = topic_name
        self.save_every_nth_image = save_every_nth_image
        # Create the folder if it doesn't exist
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)

        # Subscribe to the RGB image topic
        rospy.Subscriber(self.topic_name, Image, self.image_callback)

    def image_callback(self, msg):
        try:
            if self.image_count % self.save_every_nth_image == 0:
                # Convert the ROS image to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

                # Save the image to the folder
                image_filename = f"image_{self.image_count}.jpg"
                image_path = os.path.join(self.image_folder, image_filename)
                cv2.imwrite(image_path, cv_image)

                rospy.loginfo(f"Image saved: {image_path}")

            self.image_count += 1
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

def run_imagesaver():
    # init Ros node
    rospy.init_node('image_saver', anonymous=True)
    # specify topic name and which images to save.
    topic_name = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
    save_every_nth_image = 5
    # create ImageSaver instance.
    image_saver = ImageSaver(topic_name, save_every_nth_image)
    # run
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    run_imagesaver()
