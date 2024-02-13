from play_rosbag import play_rosbag
from get_images_from_rosbag import run_imagesaver
import os
import pathlib

# Get the file paths of the rosbags we want to play
curr_dir = pathlib.Path(__file__).parent.resolve()
rosbag_dir_file_path = curr_dir / "darknet_bags" / "bags"
rosbags = os.listdir(rosbag_dir_file_path)
print(f"rosbags found: {rosbags}")


# iteratively play the respective rosbag 
for rosbag in rosbags:
    print(f"playing rosbag {rosbag}")
    play_rosbag(rosbag_dir_file_path / rosbag)
