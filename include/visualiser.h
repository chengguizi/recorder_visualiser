#ifndef VISUALISER_H
#define VISUALISER_H
/**
 * @file visualiser.h
 * @author Huimin Cheng (NUS)
 * @brief 
 * @version 0.1
 * @date 2018-11-01
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <opencv2/core.hpp>
#include <DUtilsCV/DUtilsCV.h>

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>

#include <string>
#include <vector>

class TrajectoryVisualiser{

public:

    // typedef DUtilsCV::Drawing::Plot::Style('k',2, cv::LINE_AA) normal_style; // thickness
	// typedef DUtilsCV::Drawing::Plot::Style('r', 2, cv::LINE_AA) loop_style; // color, thickness

    enum TrajectoryName{
        IMU = 0,
        IMU_HEURISTIC,
        VO,
        EKF,
        TRAJECTORYNAME_SIZE
    };

    struct TrajectorySettings{
        std::string name;
        std::string ros_topic;
        // Eigen::Matrix3d R_pw = Eigen::Matrix3d::Identity(); // this pose frame in world frame

        DUtilsCV::Drawing::Plot::Style style = DUtilsCV::Drawing::Plot::Style('k',1, cv::LINE_AA);
        cv::MarkerTypes marker =  cv::MARKER_SQUARE;
    };

    struct Parameters{
        int width, height;
        int umin, umax, vmin, vmax;
        TrajectorySettings settings[TRAJECTORYNAME_SIZE];
        std::string imu_ros_topic = "/imu0";
    };

    void setParams(const Parameters &params);

    // Add one point to the line, specified by the name
    void addPoint(const TrajectoryName &name, const cv::Point2d &pt);

    void clearPlot(){
        for (int i = 0; i < (int)TRAJECTORYNAME_SIZE; i++)
            trajectories[i].clear();

        createPlot(params.width,params.height,params.umin,params.umax,params.vmin,params.vmax);
        drawGrid();
        drawLabel();    
        last_stamp = ros::Time(0);
    }

    cv::Mat getImagewithStamp();
    

private:
    Parameters params;

    DUtilsCV::Drawing::Plot implot;

    std::vector<cv::Point2d> trajectories[TRAJECTORYNAME_SIZE];
    
    ros::Subscriber _reset_sub;
    ros::Subscriber _pose_sub[TRAJECTORYNAME_SIZE];
    // ros::Subscriber _imu_sub;
    ros::Publisher _trajectory_pub;
    
    // Eigen::Quaternion<double> init_imu_q;
    ros::Time last_stamp;

    int ignore_n = 0;

    void subscribeTopics(){

        ros::NodeHandle nh;

        for (int i = 0; i < (int)TRAJECTORYNAME_SIZE; i++){
            if (params.settings[i].ros_topic.empty())
                continue;
            _pose_sub[i] = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(params.settings[i].ros_topic, 1, 
                    boost::bind(&TrajectoryVisualiser::poseCallback, this, _1, i) );
            ROS_INFO_STREAM(params.settings[i].name << "is subscribed to " << params.settings[i].ros_topic);
        }

        
        _reset_sub = nh.subscribe("/reset",3, &TrajectoryVisualiser::resetCallback, this);
        ROS_INFO_STREAM("Subscribed to /reset topic");

        // _imu_sub = nh.subscribe(imu_ros_topic,3, &TrajectoryVisualiser::imuCallback, this);
        // ROS_INFO_STREAM("Subscribed to " << imu_ros_topic);
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg, int idx);

    void resetCallback(const std_msgs::HeaderConstPtr &header);

    // void imuCallback(const sensor_msgs::ImuConstPtr &imu);

    // u-v coordinates, u is horizontal
    void createPlot(int width, int height, int umin, int umax, int vmin, int vmax);
    void drawGrid();
    void drawLabel();
    
    void publish(const cv::Mat &img, const ros::Time &stamp);
};

void TrajectoryVisualiser::setParams(const Parameters &params){
    
    this->params = params;
    
    
    // Initialise publisher
    ros::NodeHandle local_nh("~");
    _trajectory_pub = local_nh.advertise<sensor_msgs::Image>("trajectories",3);
    
    subscribeTopics();
    
    clearPlot();

    ROS_INFO("Created New Plot");
    
    publish(getImagewithStamp(), ros::Time(0));

    ROS_INFO("TrajectoryVisualiser(): Parameters Loaded.");
}

void TrajectoryVisualiser::createPlot(int width, int height, int umin, int umax, int vmin, int vmax){
    implot.create(width,height,umin,umax,vmin,vmax);
}

void TrajectoryVisualiser::addPoint(const TrajectoryName &name, const cv::Point2d &pt){
    int idx = (int)name;
    // ROS_INFO_STREAM_THROTTLE(0.5,"Add point: " << pt.x << "," << pt.y << " for " << params.settings[idx].name);
    
    if (trajectories[idx].size() < 2)
        trajectories[idx].push_back(pt);
    else{
        trajectories[idx].erase(trajectories[idx].begin());
        trajectories[idx].push_back(pt);
    }

    if (trajectories[idx].size() > 1){
        auto size = trajectories[idx].size();
        cv::Point2d prevPt = trajectories[idx][size-2];
        cv::Point2d currPt = trajectories[idx][size-1];
        implot.line(prevPt.x, prevPt.y, currPt.x, currPt.y, params.settings[idx].style);
    }
}

void TrajectoryVisualiser::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg, int idx){

    if (ignore_n){
        assert(trajectories[(int)EKF].size() == 0);
        ROS_INFO_STREAM("Ignoring " << ignore_n <<"frames");
        ignore_n--;
        return;
    }
        
    
    cv::Point2d pt;
    // Eigen::Vector3d pt3d = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};

    // // Transformation of coordinate system
    // pt3d = params.settings[idx].R_pw * pt3d;

    // pt.x = pt3d.x; // East direction in ENU
    // pt.y = pt3d.y; // North direction in ENU

    pt.x = msg->pose.pose.position.x; // This assume all pose received are in common world frame (e.g. ENU)
    pt.y = -msg->pose.pose.position.y;
    
    auto stamp = msg->header.stamp;
    addPoint((TrajectoryName)idx,pt);

    if (last_stamp < stamp){
        
        cv::Mat outImg = getImagewithStamp();
        // publish the trajectories image
        publish(outImg, stamp);
    }
}

void TrajectoryVisualiser::resetCallback(const std_msgs::HeaderConstPtr &header){
    ROS_WARN_STREAM("Reset occurs at " << header->stamp );
    clearPlot();
    ignore_n = 2;
    cv::Mat outImg = getImagewithStamp();
    publish(outImg, ros::Time(0));
}

// void TrajectoryVisualiser::imuCallback(const sensor_msgs::ImuConstPtr &imu){

// }

cv::Mat TrajectoryVisualiser::getImagewithStamp(){
    cv::Mat outImg = implot.getImage().clone();
    cv::putText(outImg, std::to_string(last_stamp.toNSec()), cv::Point(5 /*column*/ ,10 /*row*/), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(200,200,200));
    
    // Also draw a cross shape on the current location for each trajectory
    for (int i=0; i<(int)TRAJECTORYNAME_SIZE; i++){

        if (trajectories[i].empty())
            continue;
        cv::Point2d pt = trajectories[i].back();
        int x = implot.toPxX(pt.x);
        int y = implot.toPxY(pt.y);
        cv::drawMarker(outImg, cv::Point(x,y), params.settings[i].style.color, 
            params.settings[i].marker, 10, 1, CV_AA);
    }
    
    return outImg;
}

void TrajectoryVisualiser::drawLabel(){
    int height_offset = 10;
    int width_offset_label = params.width - 100;
    // int width_offset_line = params.width - 20;

    cv::Mat refImg = implot.getImage();

    for(int i=0; i<(int)TRAJECTORYNAME_SIZE; i++){
        int offset = i*10;
        cv::putText(refImg, params.settings[i].name, cv::Point(width_offset_label /*column*/ , height_offset + offset /*row*/), 
            cv::FONT_HERSHEY_SIMPLEX, 0.35, params.settings[i].style.color);
    }

}

void TrajectoryVisualiser::drawGrid(){

    int start_u = std::ceil(params.umin);
    int end_u = std::floor(params.umax);
    int start_v = std::ceil(params.vmin);
    int end_v = std::floor(params.vmax);
    cv::Mat refImg = implot.getImage();

    cv::Point2i p1, p2;
    // draw x-axis
    p1.x = 0; // column
    p1.y = params.height / 2;

    p2.x = params.width-1; // column
    p2.y = params.height / 2;
    cv::arrowedLine(refImg, p1, p2 , cv::Scalar(200,200,200),1,CV_AA,0,0.02);

    // draw y-axis
    p1.x = params.width / 2; // column
    p1.y = params.height - 1;

    p2.x = params.width / 2; // column
    p2.y = 0;
    cv::arrowedLine(refImg, p1, p2 , cv::Scalar(200,200,200),1,CV_AA,0,0.02);

    DUtilsCV::Drawing::Plot::Style gridLine(cv::Scalar(230,230,230),0.5, cv::LINE_AA);

    for (int vert = start_u ; vert <= end_u; vert=vert+2)
        implot.line(vert, params.vmin, vert, params.vmax, gridLine);

    for (int hori = start_v ; hori <= end_v; hori=hori+2)
        implot.line(params.umin, hori, params.umax, hori, gridLine);

    cv::putText(refImg, "N", cv::Point( params.width/2 /*column*/ , 20 /*row*/), 
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(20,20,20));

    cv::putText(refImg, "E", cv::Point( params.width - 20 /*column*/ , params.height/2 /*row*/), 
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(20,20,20));
    
}

void TrajectoryVisualiser::publish(const cv::Mat &img, const ros::Time &stamp){

    // static double last_publish_time = 0;

    if ( (stamp - last_stamp).toSec() < 0.08 && !stamp.isZero()) // dont publish too frequently
        return;

    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = "x-y plot";
    cv_bridge::CvImage cvImage = cv_bridge::CvImage(header, \
                    sensor_msgs::image_encodings::BGR8, img);
        _trajectory_pub.publish(cvImage.toImageMsg());

    // last_publish_time = ros::Time::now().toSec();

    last_stamp = stamp;
}

#endif /* VISUALISER_H */
