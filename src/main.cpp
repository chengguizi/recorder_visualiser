/**
 * @file main.cpp
 * @author Huimin Cheng (NUS)
 * @brief 
 * @version 0.1
 * @date 2018-11-01
 * 
 * @copyright Copyright (c) 2018
 * 
 */


// ROS Integration
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "visualiser.h"


int main(int argc, char **argv){

    
	// ROS Initialisation
	ros::init(argc, argv, "recorder_visualiser");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");

    TrajectoryVisualiser tv;
    TrajectoryVisualiser::Parameters tv_params;

    tv_params.settings[TrajectoryVisualiser::IMU].name = "IMU Pose";
    tv_params.settings[TrajectoryVisualiser::IMU].ros_topic = "/ekf_fusion/pose_integrated";
    tv_params.settings[TrajectoryVisualiser::IMU].style = DUtilsCV::Drawing::Plot::Style(cv::Scalar(120,120,120), 0.6, cv::LINE_AA);
    tv_params.settings[TrajectoryVisualiser::IMU].marker =  cv::MARKER_SQUARE;

    tv_params.settings[TrajectoryVisualiser::IMU_HEURISTIC].name = "IMU Heuristic Pose";
    tv_params.settings[TrajectoryVisualiser::IMU_HEURISTIC].ros_topic = "/dead_reckoning/pose";
    tv_params.settings[TrajectoryVisualiser::IMU_HEURISTIC].style = DUtilsCV::Drawing::Plot::Style('k', 0.6, cv::LINE_AA);
    tv_params.settings[TrajectoryVisualiser::IMU_HEURISTIC].marker = cv::MARKER_TRIANGLE_UP;

    tv_params.settings[TrajectoryVisualiser::VO].name = "VO Pose";
    tv_params.settings[TrajectoryVisualiser::VO].ros_topic = "/stereo_odometer/pose";
    tv_params.settings[TrajectoryVisualiser::VO].style = DUtilsCV::Drawing::Plot::Style('g', 0.6, cv::LINE_AA);
    tv_params.settings[TrajectoryVisualiser::VO].marker =  cv::MARKER_TILTED_CROSS;

    tv_params.settings[TrajectoryVisualiser::EKF].name = "Fusion Pose";
    tv_params.settings[TrajectoryVisualiser::EKF].ros_topic = "/ekf_fusion/pose_local";
    tv_params.settings[TrajectoryVisualiser::EKF].style = DUtilsCV::Drawing::Plot::Style('r', 1, cv::LINE_AA);
    tv_params.settings[TrajectoryVisualiser::EKF].marker = cv::MARKER_CROSS;

    // decide the dimension of the visualisation and the scale
    tv_params.width = tv_params.height = 512;
    // tv_params.roi_width = tv_params.roi_height = 256;
    tv_params.umin = tv_params.vmin = -64; // in meters
    tv_params.umax = tv_params.vmax = 64; // in meters

    tv_params.grid_size = 5;
    tv.setParams(tv_params);

    ros::spin();
    
}
