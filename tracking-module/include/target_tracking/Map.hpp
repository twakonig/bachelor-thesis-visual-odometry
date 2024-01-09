//
// Created by theresa on 23.04.20.
//

#ifndef TARGET_TRACKING_MAP_HPP
#define TARGET_TRACKING_MAP_HPP

#include "../include/target_tracking/MapPoint.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ctime>
#include <cmath>
#include <cxeigen.hpp>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <bits/stdc++.h>
#include <cassert>

using namespace cv;

namespace target_tracking {


class Map {
public:

    //holds all data for a point stored in the Map
    class PointData {
    public:
        PointData(const int &id, const Eigen::Vector3d &position) : landmark(id, position) { }
        target_tracking::MapPoint landmark;
        Mat pt_descriptor;
        int n_views;
    };

    //all the landmarks/points (triangulated & filtered) stored in a vector
    std::vector<PointData> registeredPoints_;

    int publicMember_;

private:

    int privateMember_;

};

}


#endif //TARGET_TRACKING_MAP_HPP
