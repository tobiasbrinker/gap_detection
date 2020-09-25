# Gap Detector
<p align='center'>
<img src="https://github.com/tobiasbrinker/gap_detection/blob/master/assets/result.png" alt="result" width="600"/>
</p>
This is the official repository for the gap detector implementation of the ROBOVIS 2020 paper "A Visual Intelligence Scheme for Hard Drive Disassembly in Automated Recycling Routines".<br>
We address the problem of analyzing the visual scenes on industrial-grade tasks, for example, automated robotic recycling of a computer hard drive with small components and little space for manipulation. The gap detector is a submodule that is responsible for the detection of gaps in devices. Given an input point cloud of the device, the gap detector produces a set of 3D outlines for the recognized gaps of the device.

## Requirements
* ROS melodic 1.14.3
* numpy 1.16.5
* scipy 1.2.2
* matplotlib 2.2.4
* scikit-learn 0.20.4
* scikit-image 0.14.3
* hdbscan 0.8.22

## Installation
1. Make sure you have set up a [ROS workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
2. Clone this repository into the source directory of your workspace e.g. `catkin_ws/src/gap_detection`
3. Install dependencies <br> `pip install -r requirements.txt`
4. Build the package from the workspace root directory <br> `catkin_make`

## Getting Started
1. Launch the gap detector on a demo point cloud <br> `roslaunch ugoe_gap_detection_ros ugoe_gap_detection_ros.launch bag:=true`
2. RVIZ and the reconfigure GUI will open. Load in the demo config by selecting the preprocessing node and pressing the  "Load from File"-Button in the upper left. Navigate to the cloned repository in the workspace and load the demo config from the demo directory(`catkin_ws/src/gap_detection/demo/demo-config.yaml`).
3. Select the gap boundaries and volumes in RVIZ to show the detected gaps(sometimes the detection process can take a while to reconfigure).

## Other options
 - Launch the detector using a bagfile: <br> `roslaunch ugoe_gap_detection_ros ugoe_gap_detection_ros.launch bag:=true bag_file:="/PATH/TO/ROSBAG.bag"`
-  Launch from a Nerian Stereo Camera: <br> `roslaunch ugoe_gap_detection_ros ugoe_gap_detection_ros.launch`
- Query the gaps from the gap detection service: <br>`rosservice call /ugoe_gap_detection_ros/detect_gaps`

## Data
The dataset and configurations, used in the evaluation of the detector, can be found [here](https://owncloud.gwdg.de/index.php/s/66olU321Hw2Fiz6). 

## Citation
```
@inproceedings{Yildiz-2020-ROBOVIS1,
title = {A Visual Intelligence Scheme for Hard Drive Disassembly in Automated Recycling Routines},
author = {Yildiz, Erenus and Tobias Brinker and Erwan Renaudo and Jakob J. Hollenstein and Simon Haller-Seeber and Justus Piater and Florentin Woergoetter},
booktitle = {International Conference on Robotics, Computer Vision and Intelligent Systems},
year = {2020},
iattachedto = {D3.2}
}
```
