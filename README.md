# Gap Detection in Hard Drive Disassembly Processes
Code repository for the bachelor thesis "Gap Detection in Hard Drive Disassembly Processes". Evaluation data used in the thesis can be found [here](https://owncloud.gwdg.de/index.php/s/66olU321Hw2Fiz6).

## Requirements
* ROS melodic 1.14.3
* numpy 1.16.5
* scipy 1.2.2
* matplotlib 2.2.4
* scikit-learn 0.20.4
* scikit-image 0.14.3
* hdbscan 0.8.22


## Launching
* Using the Nerian stereo camera: `roslaunch ugoe_gap_detection_ros ugoe_gap_detection_ros.launch`
* Using a bagfile: `roslaunch ugoe_gap_detection_ros ugoe_gap_detection_ros.launch bag:=true bag_file:="/PATH/TO/ROSBAG.bag"`

## Gap service
The currently detected gaps can be request using rosservice: `rosservice call /ugoe_gap_detection_ros/detect_gaps`

## Evaluation
1. Launch the detector and configure settings.
2. Pause the ROS process by pressing the space bar in the terminal.
3. Create a .pcd file of the denoised pointcloud for annotation by clicking the `create_PCD` checkbox in the dynamic reconfigure interface of the preprocessing node.
4. Create a report of the gap detector by clicking the `create_evaluation` checkbox in the dynamic reconfigure interface of the gap detector node. This will create a detector_evaluation.txt which contains infos about the gaps.
5. Resume the ROS process by using space again in the terminal to create the report and the .pcd file in the evaluation folder.
6. Annotate the created .pcd file by using a segmentation editor(for example: https://github.com/Hitachi-Automotive-And-Industry-Lab/semantic-segmentation-editor). Leave the first label (0.0) to the unlabeled point category.
7. Read out the labels from the annotation (also in pcd format) by using `python evaluation/evaluate_annotation.py <annotation-filename> <depth-axis>`. This will create a annotation_evaluation.txt listing the labels, the number of points and volume of each gap as well as the center point for each gap.

