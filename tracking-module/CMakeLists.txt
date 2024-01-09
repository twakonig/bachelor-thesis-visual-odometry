cmake_minimum_required(VERSION 2.8.3)
project(target_tracking)

## Use C++11
add_definitions(-std=c++11)

set(PACKAGE_DEPENDENCIES
        roscpp
        roslib
        cv_bridge
        image_transport
        sensor_msgs
        geometry_msgs
        std_msgs
        tf2
        tf2_ros
        #opencv3_catkin
        #confusion
        )


## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        ${PACKAGE_DEPENDENCIES}
        )


## Find system libraries
find_package(OpenCV REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}_core
        CATKIN_DEPENDS
            ${PACKAGE_DEPENDENCIES}
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
        src/FeatureTriangulation.cpp
        src/FeatureMatchingModule.cpp
        include/target_tracking/MapPoint.hpp)



## Declare cpp executables
add_executable(${PROJECT_NAME}
        src/feature_triangulation_node.cpp
        src/FeatureMatchingModule.cpp
        )


## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
        ${catkin_EXPORTED_TARGETS}
        )

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_core
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
       )

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "feature_triangulation_node")


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
        DIRECTORY doc
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

