//
// Created by theresa on 07.05.20.
//

#ifndef TARGET_TRACKING_MAPPOINT_HPP
#define TARGET_TRACKING_MAPPOINT_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>


namespace target_tracking {

    class MapPoint {
    public:
        //constructor, initializing private members
        MapPoint(const int &id, const Eigen::Vector3d &position_w) :
                id_(id), position_w_(position_w) { }

        int Id() const { return id_; }
        //... .data() returns address to data of Eigen::Vector3d; returns a pointer to the memory location of the first entry of the matrix.
        double *GetPositionDataPointer() { return position_w_.data(); }

        void printData() {  std::cout << "ID: " << id_ << std::endl;
                            std::cout << "Coordinates: " << std::endl;
                            std::cout << position_w_ << std::endl;  }


    private:
        int id_;
        Eigen::Vector3d position_w_; ///< The optimized 3d position. This should only be accessed from the estimation thread!!
    };

} // namespace




#endif //TARGET_TRACKING_MAPPOINT_HPP
