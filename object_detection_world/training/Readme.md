# Readme File for the darknet retraining. 
This readme file assumes that you have installed darknet ros. If that is not the case, please go to the repository readme file and follow the instructions.

If you just want to run the training, go through the setup, don't install label studio and then go to [Run the training](#Run-the-training). 

If you just want to run the custom network, go to [Prediction](#Prediction).

Otherwise if you want to use other data or change the labels or smth, go through all of it. 

## Setup

### Darknet Training tutorials for reference:

#### I mainly focued on this one:

https://medium.com/@hitesh-gupta/custom-object-detection-by-yolov3-darknet-6a804c57f276

#### Additional tutorials I also took a look at:

https://roboticsknowledgebase.com/wiki/machine-learning/train-darknet-on-custom-dataset/

https://medium.com/@thomas.lever.business/training-yolov3-convolutional-neural-networks-using-darknet-2f429858583c

https://gist.github.com/fuloporsi/52361c64019a1f4f405b41666cb052f8

https://timebutt.github.io/static/how-to-train-yolov2-to-detect-custom-objects/

https://annacsmedeiros.medium.com/from-yolo-annotation-to-using-the-weights-with-darknet-ros-2ac0f707dcdb

https://blog.francium.tech/custom-object-training-and-detection-with-yolov3-darknet-and-opencv-41542f2ff44e

http://emaraic.com/blog/yolov3-custom-object-detector


### Depending on the way you installed cuda, there are different locations and ways to install cudnn:

#### Install cudnn when cuda was installed with the following link:
https://stackoverflow.com/questions/31326015/how-to-verify-cudnn-installation

#### Install cudnn when cuda was installed using apt: 
https://askubuntu.com/questions/767269/how-can-i-install-cudnn-on-ubuntu-16-04/767270#767270


### Optionally: If you are using docker for ROS1 and need to create a new image that includes CUDA:
#### docker commit command: 
https://docs.docker.com/engine/reference/commandline/commit/

### Install label-studio
1) Open a terminal
2) Type `pip install -U label-studio`

## Data Acquisition
I made rosbags from hsrb's camera topic containing the video feed with the desired objects.
If you want to use my rosbags, these can be downloaded from [rosbags](https://drive.google.com/drive/folders/1x9WCDsTxU7X-ykubjRppJEF_jbiJNi0a?usp=sharing) and should be placed in training_helpers/darknet_bags/bags.

## Data Preprocessing
We wrote two python scripts that we used to 
1) run the rosbags (training_helpers/main.py)
2) subscribe to the video topic and save images from it (training_helpers/get_images_from_rosbag.py)
   
To create the images from the collected rosbags, these have to be run in two seperate terminals, one creates the topic and the other one subscribes to it. 
   
## Image Labeling
Once the images are saved, we have to label them. 

Optionally, you can download our labeled images from [Labeled Images](https://drive.google.com/drive/folders/1CoZcy3mNpAxPyaINh2AEKfgZAVn9NRht?usp=sharing).

1) Open a terminal.
2) Source the environment if you installed label-studio in an venv: `source env/bin/activate`
3) Set an environment variable: `export DATA_UPLOAD_MAX_NUMBER_FILES=10000000` (see https://github.com/HumanSignal/label-studio/issues/4556)
4) type `label-studio`
5) Use label-studio to create a project and start labeling the objects.
6) Export the images with the labels in "YOLO" format. Two folders will be created, one with the images and one with the labels. One additional file called "classes.txt" will be created. This specifies the labels and the order of them used in the labels files.
7) Copy the images and the labels folder into "custom_data_210123"
8) run "training_helpers/print_list.py". This will create two seperate lists in the console output. Copy the first into "train.txt" and the second into "test.txt". They list the files in the train and test dataset.
9) Copy "train.txt" and "test.txt" into "custom_data_210123"
10) Copy the folder "custom_data_210123" into the "darknet" folder.

## Training
1) All the necessary files for the custom training are located in "darknet/custom_data_210123".
2) The "cfg/yolov3-custom.cfg" file specifies the network architecture. Changes were made according to the tutorial mentioned above.
3) The "images" folder contains all the previously labeled images (copied here from previously).
4) The "labels" folder contains all the corresponding labels (copied here from preciously). 
5) "custom.names" includes the label names - should be the same as the "classes.txt"
6) "darknet53.conv.74" are the starting weights for the training. If you do not have this file, check out the previously mentioned tutorial.
7) "detector.data" includes the training configuration - adapt according to tutorial.
8) "test.txt" and "train.txt" contain a list of the respective test or train images.

### Run the training:
This obviously only works if you have labeled images. Please refer to Image Labeling to either label images or download labeled images.
1)  Navigate to darknet folder
2)  If not already there, copy "custom_data_210123" into the darknet folder.
3)  Run darknet training: `./darknet detector train custom_data_210123/detector.data custom_data_210123/cfg/yolov3-custom.cfg custom_data_210123/darknet53.conv.74`
4)  The weights will be saved in the backup folder.

## Prediction
1)  Navigate to darknet folder
2)  If not already there, copy "custom_data_210123" into the darknet folder.
3) Run with custom detection: `./darknet detector test custom_data_210123/detector.data custom_data_210123/cfg/yolov3-custom.cfg custom_data_210123/backup/yolov3-custom_100.weights -thresh 0.25 data/dog.jpg` (change the weights file and the image file accordingly)
4) Run darknet prediction using the default config and weights: `/darknet detector test cfg/coco.data cfg/yolov3.cfg ../darknet_ros/yolo_network_config/weights/yolov3.weights -thresh 0.25 data/dog.jpg`

## Additionally
Check out data augmentation options:
https://github.com/AlexeyAB/darknet/wiki/CFG-Parameters-in-the-%5Bnet%5D-section
