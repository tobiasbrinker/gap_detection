# Point Cloud Based Gap Detection in Automated Disassembly Processes To Aid Manipulation Tasks
<img src="https://github.com/tobiasbrinker/gap_detection/blob/master/assets/result.png" alt="result" width="700"/>

This is the official repository for "Point Cloud Based Gap Detection in Automated Disassembly Processes To Aid Manipulation Tasks" including an implementation of the pipeline in ROS. Evaluation data can be found [here](https://owncloud.gwdg.de/index.php/s/66olU321Hw2Fiz6). 

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
3. Select the gap boundaries and volumes in RVIZ to show the detected gaps.

## Other options
 - Launch the detector using a bagfile: <br> `roslaunch ugoe_gap_detection_ros ugoe_gap_detection_ros.launch bag:=true bag_file:="/PATH/TO/ROSBAG.bag"`
-  Launch from a Nerian Stereo Camera: <br> `roslaunch ugoe_gap_detection_ros ugoe_gap_detection_ros.launch`
- Query the gaps from the gap detection service: <br>`rosservice call /ugoe_gap_detection_ros/detect_gaps`

## Citation
