import rosbag
import rospy

def play_rosbag(bag_file_path):
    # Open the bag file
    bag = rosbag.Bag(bag_file_path, 'r')

    # Create a ROS node
    rospy.init_node('rosbag_player', anonymous=True)

    # Initialize variables for the previous message timestamp
    prev_timestamp = None

    # Use rosbag API to play the bag file
    for topic, msg, t in bag.read_messages():
        rospy.loginfo(f"Publishing message on topic {topic}")

        # Calculate sleep duration based on timestamp difference
        if prev_timestamp is not None:
            sleep_duration = (t - prev_timestamp).to_sec()
            rospy.sleep(sleep_duration)

        # Publish the message
        rospy.Publisher(topic, type(msg), queue_size=10).publish(msg)

        # Update the previous timestamp
        prev_timestamp = t

    # Close the bag file
    bag.close()

if __name__ == '__main__':
    ''' this is not used. use main.py instead, this is just for clarification of usage.'''
    bag_file_path = 'your_bag_file.bag'

    try:
        play_rosbag(bag_file_path)
    except rospy.ROSInterruptException:
        pass
