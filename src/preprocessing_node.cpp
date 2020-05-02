/**
 * Preprocessing module for the gap detector.
 * 
 * This module uses a passthrough filter to find the region of interest and applies statistical outlier removal to an
 * incoming point cloud from the Nerian stereo setup. The passthrough filter and the denoising can be adjusted using 
 * the dynamic reconfigure GUI.
 * 
 * @author Tobias Brinker
 */

#include <ros/ros.h>
#include <ros/package.h>
#include<iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
// Dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <ugoe_gap_detection_ros/preprocessingConfig.h>
// Denoising
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

// PUBLISHER AND SUBSCRIBER
ros::Publisher filtered_pub;
ros::Publisher denoised_pub;
ros::Subscriber sub;
// DYNAMIC RECONFIGURE PARAMETERS
float zLowerLimit;
float zUpperLimit;
float yLowerLimit;
float yUpperLimit;
float xLowerLimit;
float xUpperLimit;
int meanK;
float stddevMulThresh;
bool create_PCD;


/**
 * Creates a PCD file "denoised_pcd.pcd" of the current denoised point cloud 
 * in the ugoe_gap_detection_ros ROS directory in the folder evalution.
 * 
 * @param cloud Point cloud that should be saved.
 */
void create_denoised_PCD(pcl::PointCloud<pcl::PointXYZ> cloud) {
  string src_path = ros::package::getPath("ugoe_gap_detection_ros").c_str();
  string file_path = src_path + "/evaluation/denoised_pcd.pcd";
  pcl::io::savePCDFileASCII(file_path, cloud);
  string msg = "Saved " + to_string(cloud.points.size()) + " data points to " + file_path;
  ROS_INFO("%s",msg.c_str());
}

/**
 * Applies Statistical Outlier Removal to the point cloud. Parameters can be adjusted directly using dynamic reconfigure.
 *
 * @param temp_cloud_ptr Pointer to the point cloud the filter should be applied to.
 */
void statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(temp_cloud_ptr);
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(stddevMulThresh);
  sor.filter(*temp_cloud_ptr);
}

/**
 * Callback for the dynamic reconfigure package.
 */
void reconf_callback(ugoe_gap_detection_ros::preprocessingConfig &config, uint32_t level) {
  zLowerLimit = config.zLowerLimit;
  zUpperLimit = config.zUpperLimit;
  yLowerLimit = config.yLowerLimit;
  yUpperLimit = config.yUpperLimit;
  xLowerLimit = config.xLowerLimit;
  xUpperLimit = config.xUpperLimit;

  meanK = config.MeanK;
  stddevMulThresh = config.StdDevMulThresh;

  create_PCD = config.create_PCD;
  // directly uncheck checkbox to use it similar to a button
  config.create_PCD = false;
}

/**
 * Callback for the preprocessing procedure. 
 * 
 * Applies a passthrough filter and statistical outlier removal to the incoming point cloud.
 * Publishes a sensor_msgs::PointCloud2 message with the filtered cloud on topic 
 * "ugoe_gap_detection/filtered_cloud" as well as a denoised cloud on topic 
 * "ugoe_gap_detection/denoised_cloud". The denoised cloud is used by the gap detector to
 * identify gaps.
 * 
 * @param input Pointcloud2 sensor message from the "/nerian_stereo/point_cloud" topic.
 */
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  /* ----- ROSMSG CONVERSION ----- */
  sensor_msgs::PointCloud2 output;
  output = *input;
  // convert sensor-type cloud to processable-type cloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> &temp_cloud = *temp_cloud_ptr;
  pcl::fromPCLPointCloud2(pcl_pc2,temp_cloud);

  /* ----- PASSTHROUGH FILTER ----- */
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(temp_cloud_ptr);
  // filter in x-direction
  pass.setFilterFieldName("x");
  pass.setFilterLimits(xLowerLimit, xUpperLimit);
  pass.filter(temp_cloud);
  // filter in y-direction
  pass.setFilterFieldName("y");
  pass.setFilterLimits(yLowerLimit, yUpperLimit);
  pass.filter(temp_cloud);
  // filter in z-direction
  pass.setFilterFieldName("z");
  pass.setFilterLimits(zLowerLimit, zUpperLimit);
  pass.filter(temp_cloud);

  /* ----- PUBLISHING THE FILTERED CLOUD ----- */
  pcl::toPCLPointCloud2(temp_cloud, pcl_pc2);
  pcl_conversions::fromPCL(pcl_pc2, output);
  filtered_pub.publish(output);

  /* ----- DENOISING THE FILTERED CLOUD ----- */
  if(temp_cloud.empty() == true) {
    ROS_INFO("Pointcloud is empty.");
    return;
  }

  statistical_outlier_removal(temp_cloud_ptr);

  /* ----- CREATE A PCD FOR EVALUATION ----- */
  if(create_PCD) {
    create_denoised_PCD(temp_cloud);
    create_PCD = false;
  }

  /* ----- PUBLISHING THE DENOISED CLOUD ----- */
  pcl::toPCLPointCloud2(temp_cloud, pcl_pc2);
  pcl_conversions::fromPCL(pcl_pc2, output);
  denoised_pub.publish(output);
}

/**
 * Main method sets up the callbacks as well as the ROS subcriber and publisher.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "preprocessing_node");
  ros::NodeHandle nh;

  // Dynamic reconfigure configuration
  dynamic_reconfigure::Server<ugoe_gap_detection_ros::preprocessingConfig> server;
  dynamic_reconfigure::Server<ugoe_gap_detection_ros::preprocessingConfig>::CallbackType callbackBinding;
  callbackBinding = boost::bind(&reconf_callback, _1, _2);
  server.setCallback(callbackBinding);

  sub = nh.subscribe("/nerian_stereo/point_cloud", 1, cloud_cb);
  filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("ugoe_gap_detection/filtered_cloud", 1);
  denoised_pub = nh.advertise<sensor_msgs::PointCloud2>("ugoe_gap_detection/denoised_cloud", 1);

  ros::spin();
}