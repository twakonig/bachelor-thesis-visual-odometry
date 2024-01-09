
#include "target_tracking/FeatureMatchingModule.hpp"


using namespace cv;

namespace target_tracking {


    //constructor initialization
    FeatureMatchingModule::FeatureMatchingModule(ros::NodeHandle& nodeHandle)
        : nodeHandle_(nodeHandle), cache_(5), tfBuffer_(cache_), tfListener_(tfBuffer_),
            imageTransport_(nodeHandle_), countKeyFrames_(0)
    {
        //TODO: Set queue_size
        //subscribe to images
        imgSub_ = imageTransport_.subscribe("/cam0/image_rect", 2,
                                           &FeatureMatchingModule::imageCallback, this);

        //publish topic: /feature_triangulation_node/point_cloud
        pubCloud_ = nodeHandle_.advertise<sensor_msgs::PointCloud>("point_cloud", 10);

        pubObservations_ = nodeHandle_.advertise<std_msgs::Empty>("new_observations", 2);

        std::cout << "Address of featureTriangulation in FeatureMatchingModule constructor: " << &featureTriangulation_ << std::endl;
        std::cout << "[MatchAndMapModule] Initialization COMPLETE." << std::endl;

        //ROS_INFO("Successfully launched node.");

        //--------------------------------------------------------------
        /*  KAMERA MATRIX INFO -> cam_mat directly initialized in FeatureTriangulation.cpp instead
         //subscribe to camera info -> intrinsic camera matrix K
         infSubscriber_ = nodeHandle_.subscribe("/cam0/camera_info", 1000,
                 &FeatureMatchingModule::infoCallback, this);
        */
    }

    //destructor
    FeatureMatchingModule::~FeatureMatchingModule()
    {
    }


    void FeatureMatchingModule::functionForTesting() {
        std::cout << "functionForTesting was called!" << std::endl;
    }


   //gets called whenever frame comes in, takes frame as argument
    void FeatureMatchingModule::imageCallback(const sensor_msgs::ImageConstPtr& msg) {


        ROS_INFO("Back in image callback");
        //std::cout << "IMAGECallback with tf subscriber timestamp: " << msg->header.stamp.sec << std::endl;
        //---------------------------------------------------TRANSFORM DATA---------------------------------------------
        try {
            //transformation in camera frame coordinates
            worldToCam_ = tfBuffer_.lookupTransform("cam", "world", msg->header.stamp);
            imuToWorld_ = tfBuffer_.lookupTransform("world", "imu", msg->header.stamp);
            double imu_x = imuToWorld_.transform.translation.x;
            //toSec, or sec?
            double t_img = msg->header.stamp.toSec();

//            std::cout << "Translation T_c_w, imgCallback Matching: " << std::endl;
//            std::cout << worldToCam_.transform.translation << std::endl;
//            std::cout << "Quaternion T_c_w, imgCallback Matching: " << std::endl;
//            std::cout << worldToCam_.transform.rotation.x << std::endl;
//            std::cout << worldToCam_.transform.rotation.y << std::endl;
//            std::cout << worldToCam_.transform.rotation.z << std::endl;
//            std::cout << worldToCam_.transform.rotation.w << std::endl;

            featureTriangulation_.handleTransformData(worldToCam_, imu_x, t_img);
            ROS_INFO("after passing transform");
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
        //--------------------------------------------------------------------------------------------------------------

        bool isKeyFrame;
        Mat imported_frame = cv_bridge::toCvShare(msg, "mono8")->image;
        isKeyFrame = featureTriangulation_.extractFeatures(imported_frame);
        if (countKeyFrames_ > 1)
            pubObservations_.publish(observation_);

        if (isKeyFrame == true)
            countKeyFrames_ += 1;

        if ((isKeyFrame == true) && (countKeyFrames_ > 1)) {
            std::vector<Point3f> cloud_points;
            geometry_msgs::Point32 point_msg_;

            //point triangulation, retrieve 3D coordinates of keypoints
            cloud_points = featureTriangulation_.triangulateAndStore();

            //Fallthrough case if no triangulations were made
            if(cloud_points.size() == 0) {
                ROS_INFO("Exception - TOO FEW NEW POINTS FOR TRIANGULATION");
                return;
            }

            //cloud_msg_.points.clear();

            for (int i = 0; i < cloud_points.size(); ++i) {

                point_msg_.x = cloud_points[i].x;
                point_msg_.y = cloud_points[i].y;
                point_msg_.z = cloud_points[i].z;
                //fill PointCloud
                cloud_msg_.points.push_back(point_msg_);
            }

            cloud_msg_.header.stamp = msg->header.stamp;
            cloud_msg_.header.frame_id = "world";
            pubCloud_.publish(cloud_msg_);

            ROS_INFO("TRIANGULATION");
            return;
        } else {
            ROS_INFO("NO TRIANGULATION");
            return;
        }

    }


//----------------------------------------------------------------------------------------------------------------------
// void handleTransformDataGen(Eigen::Quaternion<double> rotation, Eigen::Matrix<double, 3, 1> translation, double x_imu, double t_frame);
    void FeatureMatchingModule::generalCallback(const sensor_msgs::ImageConstPtr &msg, Eigen::Quaternion<double> rotation, Eigen::Matrix<double, 3, 1> translation, double imu_x_w) {

//            std::cout << "IMU & IMG Callback timestamp: " << msg->header.stamp.sec << std::endl;
//            std::cout << "Translation T_c_w, directly from tracking module: " << std::endl;
//            std::cout << translation << std::endl;
//            std::cout << "Quaternion T_c_w, directly from tracking module: " << std::endl;
//            std::cout << rotation.x() << std::endl;
//            std::cout << rotation.y() << std::endl;
//            std::cout << rotation.z() << std::endl;
//            std::cout << rotation.w() << std::endl;


//transform for using with featureTriangulation

/*
        ROS_INFO("Back in GENERAL callback");
        //std::cout << "IMAGECallback with tf subscriber timestamp: " << msg->header.stamp.sec << std::endl;
        //---------------------------------------------------TRANSFORM DATA---------------------------------------------
        try {
            //toSec, or sec?
            double t_img = msg->header.stamp.toSec();
            featureTriangulation_.handleTransformDataGen(rotation, translation, imu_x_w, t_img);
            ROS_INFO("after passing transform");
        }
        catch (std::exception &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
        //--------------------------------------------------------------------------------------------------------------

        bool isKeyFrame;
        Mat imported_frame = cv_bridge::toCvShare(msg, "mono8")->image;
        isKeyFrame = featureTriangulation_.extractFeatures(imported_frame);
        if (countKeyFrames_ > 1)
            pubObservations_.publish(observation_);

        if (isKeyFrame == true)
            countKeyFrames_ += 1;

        if ((isKeyFrame == true) && (countKeyFrames_ > 1)) {
            std::vector<Point3f> cloud_points;
            geometry_msgs::Point32 point_msg_;

            //point triangulation, retrieve 3D coordinates of keypoints
            cloud_points = featureTriangulation_.triangulateAndStore();

            //Falthrough case if no triangulations were made
            if(cloud_points.size() == 0) {
                ROS_INFO("Exception - TOO FEW NEW POINTS FOR TRIANGULATION");
                return;
            }

            //cloud_msg_.points.clear();

            for (int i = 0; i < cloud_points.size(); ++i) {

                point_msg_.x = cloud_points[i].x;
                point_msg_.y = cloud_points[i].y;
                point_msg_.z = cloud_points[i].z;
                //fill PointCloud
                cloud_msg_.points.push_back(point_msg_);
            }

            cloud_msg_.header.stamp = msg->header.stamp;
            cloud_msg_.header.frame_id = "world";
            pubCloud_.publish(cloud_msg_);

            ROS_INFO("TRIANGULATION");
            return;
        } else {
            ROS_INFO("NO TRIANGULATION");
            return;
        }
*/
    }


    /*
    void FeatureMatchingModule::infoCallback(const sensor_msgs::CameraInfoConstPtr& info) {

        if (numFrame_ < 1) {
            Mat K = (Mat_<double>(3, 3) <<
                                        info->K[0], info->K[1], info->K[2],
                                        info->K[3], info->K[4], info->K[5],
                                        info->K[6], info->K[7], info->K[8]);

            featureTriangulation_.getKmatrix(K);
        }
        ROS_INFO("infoCallback worked");
    }
    */


} /* namespace */
