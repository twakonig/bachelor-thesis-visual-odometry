
#ifndef TARGET_TRACKING_FEATURETRIANGULATION_HPP
#define TARGET_TRACKING_FEATURETRIANGULATION_HPP


#include "../include/target_tracking/Map.hpp"

#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>                   //for KeyPoint
#include <opencv2/features2d.hpp>                   //for ORB keypoint detector
#include <sensor_msgs/CameraInfo.h>                 //for sensor_msgs
#include <sensor_msgs/PointCloud.h>
#include <image_transport/image_transport.h>        //for ImageConstPtr&
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <ctime>
#include <cxeigen.hpp>
#include <unordered_map>
#include <bits/stdc++.h>
#include <cassert>


using namespace cv;


namespace target_tracking {

/*!
 * Class containing the algorithmic part of the package.
 */

    class FeatureTriangulation {
    public:
        //constructor
        FeatureTriangulation();

        //destructor
        virtual ~FeatureTriangulation();

        //METHODS of FeatureTriangulation class
        void handleTransformData(geometry_msgs::TransformStamped world_cam, double x_imu, double t_frame);

        void handleTransformDataGen(Eigen::Quaternion<double> rotation, Eigen::Matrix<double, 3, 1> translation, double x_imu, double t_frame){
            incoming_.transl_eigen = translation;
            incoming_.rotation_eigen = rotation;
            incoming_.imu_x = x_imu;
            incoming_.time = t_frame;
        }

        bool extractFeatures(Mat frame);

        bool decideToStoreImage();

        void matchFeatures();

        void matchToMap();

        std::vector<Point3f> triangulateAndStore();

        std::vector<DMatch> matchDescriptors(Mat query_descr, Mat train_descr, float thresh);

        void getRotAndProjMat();

        std::vector<Point3f> checkBackProjection(std::vector<Point3f> pcl_unfiltered, std::vector<Point2f> pts_currentkf);

        //void getKmatrix(Mat intrinsic_data);

        //internal variable to hold current amount of keyframes -> descriptors only calculated for keyframes
        int nKeyframes_;

        Map map_;

        //observation: 2D feature coordinates of keypoint
        struct MatchedObs {
            double t_sec;
            std::vector<int> id_list;       //indexes of points in map they were matched to
            std::vector<Point2f> coord_2d;
            //int id;
            //Point2f coord_2d;
        };

        MatchedObs matchedObservations_;



    private:

        //internal variable to hold current amount of keyframes -> descriptors only calculated for keyframes
       // int nKeyframes_;

        //if true, first frame
        bool firstFrame_;

        //for inspection, number of times frame is queried
        int countFrames_;

        //true if incoming frame is declared a keyframe -> passed to FeatureMatchingModule
        bool isKeyframe_;

        //Mat cam_mat_;

        struct KeyFrame {
            //all data needed per keyframe
            geometry_msgs::TransformStamped worldToCam;
            double imu_x;
            double time;
            Eigen::Vector3d transl_eigen;
            Eigen::Quaternion<double> rotation_eigen;
           // Eigen::Matrix4d transformation_mat;
            std::vector<KeyPoint> keypoints;
            Mat descriptor;
            //for triangulation
            Mat rot_mat;
            Mat proj_mat;
        };

        //not keyframe, actually just frame
        KeyFrame incoming_;

        //real keyframes
        KeyFrame currentkf_;
        KeyFrame previouskf_;

        std::vector<DMatch> good_matches_;

        int counter_;

        int all_points_;
        int filtered_points_;


        Mat getPtsToMatch();

        Mat checkIfNew(Mat initial_matches);

        std::vector<int> findNewKeypts(std::vector<DMatch> already_present, std::vector<int> indexes_initial);

        void insertKeyPoints(Mat matched_descr, std::vector<Point3f> pts_filtered);

        //number of elements in keypoint list BEFORE adding new ones
        int n_;
        std::vector<int> new_points_;
        std::vector<int> indexes_filtered_;

    };

} /* namespace */

#endif //TARGET_TRACKING_FEATURETRIANGULATION_HPP

