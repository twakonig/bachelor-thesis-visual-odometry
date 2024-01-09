
#ifndef TARGET_TRACKING_FEATUREMATCHINGMODULE_HPP
#define TARGET_TRACKING_FEATUREMATCHINGMODULE_HPP

#include "../include/target_tracking/FeatureTriangulation.hpp"

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>             //for sensor_msgs
#include <image_transport/image_transport.h>    //for ImageConstPtr&
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <exception>
//#include "confusion/utilities/Pose.h"



namespace target_tracking {


    class FeatureMatchingModule {
    public:
        //constructorFeatureTriangulation
        FeatureMatchingModule(ros::NodeHandle& nodeHandle);

        //destructor
        virtual ~FeatureMatchingModule();

        void functionForTesting();

        FeatureTriangulation featureTriangulation_;

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        void generalCallback(const sensor_msgs::ImageConstPtr &msg, Eigen::Quaternion<double> rotation, Eigen::Matrix<double, 3, 1> translation, double imu_x_w);


    private:

        //void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        //! ROS node handle, private
        ros::NodeHandle& nodeHandle_;

        ros::Duration cache_;

        tf2_ros::Buffer tfBuffer_;

        tf2_ros::TransformListener tfListener_;

        int countKeyFrames_;

        //! ROS image_topic subscriber
        image_transport::Subscriber imgSub_;

        image_transport::ImageTransport imageTransport_;

        sensor_msgs::PointCloud cloud_msg_;
        std_msgs::Empty observation_;

        geometry_msgs::TransformStamped worldToCam_;
        geometry_msgs::TransformStamped imuToWorld_;


        //! ROS publisher
        ros::Publisher pubCloud_;
        ros::Publisher pubObservations_;

        //! FeatureTriangulation computation object
        //FeatureTriangulation featureTriangulation_;


        //void infoCallback(const sensor_msgs::CameraInfoConstPtr& info);
        //! ROS info topic subscriber (camera matrix)
        //ros::Subscriber infSubscriber_;
    };

} /* namespace */

#endif //TARGET_TRACKING_FEATUREMATCHINGMODULE_HPP
