import os
import pathlib
from pathlib import Path
import numpy as np
import shutil
import random
 
def create_list(path, train_test_split):
    file_dir = Path(__file__).parent.resolve()
    # get names of files in images directory    
    
    total_image_names = []
    # get all files in the directory
    items = os.listdir(path)
    # append all images to a list
    [total_image_names.append(f"custom_data_210123/images/{item}") for item in items if item.split(".")[1]=="jpg"]
    train_image_names = []
    test_image_names = []
    
    # shuffle
    random.shuffle(total_image_names)
    
    # split into train, test
    split_index = int(len(total_image_names) * train_test_split)
    train_image_names = total_image_names[:split_index]
    test_image_names = total_image_names[split_index:]
    
    print("#"*100)
    print(f"Train Image Names:")
    train_image_names = "\n".join(train_image_names)
    print(train_image_names)
    
    # write image names to file
    with open(file_dir / "train.txt", "w") as f:
        f.write(train_image_names)

    
    print("#"*100)
    print(f"Test Image Names:")
    test_image_names = "\n".join(test_image_names)
    print(test_image_names)
    
    with open(file_dir / "test.txt", "w") as f:
        f.write(test_image_names)

        

if __name__ == "__main__":
    train_test_split=0.9
    
    # if you want to create a list based on the images in the darknet image directory:
    #darknet_ros_dir = Path(__file__).parent.parent.parent.resolve()
    #create_list(darknet_ros_dir / "darknet/custom_data_210123/images",train_test_split)
    
    # if you want to create a list based on the images in the "saved_images" dir
    file_dir = Path(__file__).parent.resolve()
    create_list(file_dir / "saved_images",train_test_split)

