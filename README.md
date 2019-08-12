# YOLO ROS: Real-Time Object Detection for ROS

## Notes about this fork

The following has been modified in this fork (compared to v1.1.4 of the original repo):
* darknet has been updated to the current latest commit (darknet points to a [pull request](https://github.com/pjreddie/darknet/compare/master...9b7afacu) that fixes the calling of a decaprecated cuda function)
* The yolo threshold parameter is now dynamically reconfigurable
* Printing of messages now uses ROS Debug, Info, and Warn
* Messages are published in the frame that they are recieved in
* Limited detection to only when an image is recieved (reduces wasted GPU usage)

### Why use this fork?

* The commit of darknet used on the original repo has some occasional seg faults whereas the current latest commit of darknet doesn't seem to have that problem
* Increase in fps by ~10 (based on testing with a GTX 1050 Ti) when detection image is **not** being visualised

### Why not use this fork?

* This fork does drastically worse when visualising the detection image (runs at ~50-60 fps compared to ~75-85 fps when using the original repo - based on testing with a GTX 1050 Ti)

### Other comments

In conclusion, if you don't need to visualise the detection image in your application and an increase in fps is crucial, or are plagued by the seg faults in the original repo, use this fork. 

Pull requests are very welcome (especially ones that incorporate [REPs](https://github.com/ros-infrastructure/rep))!


## Overview

This is a ROS package developed for object detection in camera images. You only look once (YOLO) is a state-of-the-art, real-time object detection system. In the following ROS package you are able to use YOLO (V3) on GPU and CPU. The pre-trained model of the convolutional neural network is able to detect pre-trained classes including the data set from VOC and COCO, or you can also create a network with your own detection objects. For more information about YOLO, Darknet, available training data and training YOLO see the following link: [YOLO: Real-Time Object Detection](http://pjreddie.com/darknet/yolo/).

The YOLO packages have been tested under ROS Melodic + Ubuntu 18.04, and ROS Kinetic + Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Author of this fork: Vivek Raja, s1864074@ed.ac.uk**

**Original Author: [Marko Bjelonic](http://www.markobjelonic.me), marko.bjelonic@mavt.ethz.ch**

![Darknet Ros example: Detection image](darknet_ros/doc/test_detection.png)
![Darknet Ros example: Detection image](darknet_ros/doc/test_detection_anymal.png)

Based on the [Pascal VOC](https://pjreddie.com/projects/pascal-voc-dataset-mirror/) 2012 dataset, YOLO can detect the 20 Pascal object classes:

- person
- bird, cat, cow, dog, horse, sheep
- aeroplane, bicycle, boat, bus, car, motorbike, train
- bottle, chair, dining table, potted plant, sofa, tv/monitor

Based on the [COCO](http://cocodataset.org/#home) dataset, YOLO can detect the 80 COCO object classes:

- person
- bicycle, car, motorbike, aeroplane, bus, train, truck, boat
- traffic light, fire hydrant, stop sign, parking meter, bench
- cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe
- backpack, umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball, kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket
- bottle, wine glass, cup, fork, knife, spoon, bowl
- banana, apple, sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake
- chair, sofa, pottedplant, bed, diningtable, toilet, tvmonitor, laptop, mouse, remote, keyboard, cell phone, microwave, oven, toaster, sink, refrigerator, book, clock, vase, scissors, teddy bear, hair drier, toothbrush

## Citing

The YOLO methods used in this software are described in the paper: [You Only Look Once: Unified, Real-Time Object Detection](https://arxiv.org/abs/1506.02640).

If you are using YOLO V3 for ROS, please add the following citation to your publication:

M. Bjelonic
**"YOLO ROS: Real-Time Object Detection for ROS"**,
URL: https://github.com/leggedrobotics/darknet_ros, 2018.

    @misc{bjelonicYolo2018,
      author = {Marko Bjelonic},
      title = {{YOLO ROS}: Real-Time Object Detection for {ROS}},
      howpublished = {\url{https://github.com/leggedrobotics/darknet_ros}},
      year = {2016--2018},
    }

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionally, YOLO for ROS depends on following software:

- [OpenCV](http://opencv.org/) (computer vision library),
- [boost](http://www.boost.org/) (c++ library),

### Building

In order to install darknet_ros, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone --recursive https://github.com/vivkr/darknet_ros.git
    cd ../
    catkin build darknet_ros

Note that the first time takes long to build as the `yolov2-tiny` and `yolov3` weights are downloaded (you can skip this by commenting out the relevant lines in `darknet_ros/CMakeLists.txt`).

Darknet on the CPU is fast (approximately 1.5 seconds on an Intel Core i7-6700HQ CPU @ 2.60GHz × 8) but it's like 500 times faster on GPU! You'll have to have an Nvidia GPU and you'll have to install CUDA. The CMakeLists.txt file automatically detects if you have CUDA installed or not. CUDA is a parallel computing platform and application programming interface (API) model created by Nvidia. If you do not have CUDA on your System the build process will switch to the CPU version of YOLO. If you are compiling with CUDA, you might receive the following build error:

    nvcc fatal : Unsupported gpu architecture 'compute_61'.

This means that you need to check the compute capability (version) of your GPU. You can find a list of supported GPUs in CUDA here: [CUDA - WIKIPEDIA](https://en.wikipedia.org/wiki/CUDA#Supported_GPUs). Simply find the compute capability of your GPU and add it into darknet_ros/CMakeLists.txt. Simply add a similar line like

    -O3 -gencode arch=compute_62,code=sm_62

### Minimal setup

To test this node with Tiny YOLOv2, set the topic which your camera images are being published to in `darknet_ros/config/ros.yaml` and run `roslaunch darknet_ros darknet_ros.launch`. You can view the detection image with rviz or with `rosrun image_view image_view image:=/darknet_ros/detection_image`, or by setting `others/enable_opencv` to true in `darknet_ros/config/ros.yaml`.

### Download weights

The yolo-voc.weights and tiny-yolo-voc.weights are downloaded automatically in the CMakeLists.txt file. If you need to download them again, go into the weights folder and download the two pre-trained weights from the COCO data set:

    cd catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/weights/
    wget http://pjreddie.com/media/files/yolov2.weights
    wget http://pjreddie.com/media/files/yolov2-tiny.weights

And weights from the VOC data set can be found here:

    wget http://pjreddie.com/media/files/yolov2-voc.weights
    wget http://pjreddie.com/media/files/yolov2-tiny-voc.weights

And the pre-trained weight from YOLO v3 can be found here:

    wget http://pjreddie.com/media/files/yolov3-voc.weights
    wget http://pjreddie.com/media/files/yolov3.weights

### Use your own detection objects

In order to use your own detection objects you need to provide your weights and your cfg file inside the directories:

    catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/weights/
    catkin_workspace/src/darknet_ros/darknet_ros/yolo_network_config/cfg/

In addition, you need to create your config file for ROS where you define the names of the detection objects. You need to include it inside:

    catkin_workspace/src/darknet_ros/darknet_ros/config/

Then in the launch file you have to point to your new config file in the line:

    <rosparam command="load" ns="darknet_ros" file="$(find darknet_ros)/config/your_config_file.yaml"/>


## Basic Usage

In order to get YOLO ROS: Real-Time Object Detection for ROS to run with your robot, you will need to adapt a few parameters. It is the easiest if duplicate and adapt all the parameter files that you need to change from the `darkned_ros` package. These are specifically the parameter files in `config` and the launch file from the `launch` folder.

## Nodes

### Node: darknet_ros

This is the main YOLO ROS: Real-Time Object Detection for ROS node. It uses the camera measurements to detect pre-learned objects in the frames.

### ROS related parameters

You can change the names and other parameters of the publishers, subscribers and actions inside `darkned_ros/config/ros.yaml`.

#### Subscribed Topics

* **`camera_reading`** ([sensor_msgs/Image])

    The camera measurements.

#### Published Topics

* **`object_detector`** ([std_msgs::Int8])

    Publishes the number of detected objects.

* **`bounding_boxes`** ([darknet_ros_msgs::BoundingBoxes])

    Publishes an array of bounding boxes that gives information of the position and size of the bounding box in pixel coordinates.

* **`detection_image`** ([sensor_msgs::Image])

    Publishes an image of the detection image including the bounding boxes.

#### Actions

* **`camera_reading`** ([sensor_msgs::Image])

    Sends an action with an image and the result is an array of bounding boxes.

#### Others

* **`use_camera_msg_timestamp_for_result`** (bool)

    Whether to use the timestamp of the `camera_reading` messgae for the timestamp of the result bounding box and detection image messages.


* **`enable_opencv`** (bool)

    Enable or disable the open cv view of the detection image including the bounding boxes. The detection image can also be viewed by visualising the `detection_image` topic (`rosrun image_view image_view image:=/darknet_ros/detection_image` or with rviz); however, the node runs faster if opencv is used to visualize the detection image.

* **`wait_key_delay`** (int)

    Wait key delay in ms of the open cv window (only use if `enable_opencv` is `true`).

### Detection related parameters

You can change the parameters that are related to the detection by adding a new config file that looks similar to `darkned_ros/config/yolo.yaml`.

* **`yolo_model/config_file/name`** (string)

    Name of the cfg file of the network that is used for detection. The code searches for this name inside `darkned_ros/yolo_network_config/cfg/`.

* **`yolo_model/weight_file/name`** (string)

    Name of the weights file of the network that is used for detection. The code searches for this name inside `darkned_ros/yolo_network_config/weights/`.

* **`yolo_model/threshold/value`** (float)

    Threshold of the detection algorithm. It is defined between 0 and 1.

* **`yolo_model/detection_classes/names`** (array of strings)

    Detection names of the network used by the cfg and weights file inside `darkned_ros/yolo_network_config/`.
