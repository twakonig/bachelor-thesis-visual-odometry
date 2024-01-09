
#include "../include/target_tracking/FeatureTriangulation.hpp"


namespace target_tracking {

    //constructor
    FeatureTriangulation::FeatureTriangulation()
        : nKeyframes_(0), firstFrame_(true), countFrames_(0), isKeyframe_(true), counter_(0), all_points_(0),
        filtered_points_(0)
    {
    }

    //destructor
    FeatureTriangulation::~FeatureTriangulation()
    {
    }

    /*  MANUALLY

    void FeatureTriangulation::getKmatrix(Mat intrinsic_data) {
        cam_mat_ = intrinsic_data;
    }

     */


    void FeatureTriangulation::handleTransformData(geometry_msgs::TransformStamped world_cam, double x_imu, double t_frame) {
        //get transformation matrix
        incoming_.worldToCam = world_cam;
        incoming_.transl_eigen << incoming_.worldToCam.transform.translation.x,
                                  incoming_.worldToCam.transform.translation.y,
                                  incoming_.worldToCam.transform.translation.z;

        incoming_.imu_x = x_imu;
        incoming_.time = t_frame;
    }


/*
    void FeatureTriangulation::handleTransformDataGen(Eigen::Quaternion<double> rotation, Eigen::Matrix<double, 3, 1> translation, double x_imu, double t_frame) {

        incoming_.transl_eigen = translation;
        incoming_.rotation_eigen = rotation;
        incoming_.imu_x = x_imu;
        incoming_.time = t_frame;
    }
*/




    bool FeatureTriangulation::extractFeatures(Mat frame) {

        countFrames_ += 1;

        //BRISK detector
        Ptr<BRISK> brisk_detector = BRISK::create(70, 0, 1.0f);

        //possible to include mask where to look for keypoints
        brisk_detector -> detectAndCompute(frame, noArray(), incoming_.keypoints, incoming_.descriptor);
        std::cout << "number of keypoints: " << incoming_.keypoints.size() << std::endl;

        //OPTIONAL drawing
        //Mat img_keypoints;
        //drawKeypoints(frame, incoming_.keypoints, img_keypoints, Scalar(255,0,255), 0);    //pink

        //after first frame, for determining whether new keyframe
        if(countFrames_ > 1) {
            matchFeatures();
        }

        //after first triangulation
        if (nKeyframes_ > 2)
            matchToMap();

        bool key_criteria = decideToStoreImage();

        if (key_criteria == true) {                 //criteria for keyframe must be met
            //sets firstFrame_ to false after first call
            firstFrame_ = false;
            isKeyframe_ = true;
            //RESET PREVIOUS AND CURRENT (neglectable for first call)
            if (countFrames_ > 1) {
                previouskf_ = currentkf_;
            }
            //DECLARE INCOMING FRAME AS CURRENT KEYFRAME
            currentkf_ = incoming_;
            //compute RotMat and ProjMat of current Keyframe
            getRotAndProjMat();
        }
        return isKeyframe_;
    }




    bool FeatureTriangulation::decideToStoreImage() {

        std::cout << "number of frame queries: " << countFrames_ << std::endl;
        std::cout << "number of keyframes: " << nKeyframes_ << std::endl;

        if (firstFrame_ == true) {
            nKeyframes_ += 1;
            return true;
        }
        else if (good_matches_.size() < 90) {
            nKeyframes_ += 1;
            return true;
        }
        else {
            isKeyframe_ = false;
            return false;
        }
    }



    void FeatureTriangulation::matchFeatures() {

        //container for matches
        std::vector<std::vector<DMatch> > knn_matches;     //2D vector
        Ptr<DescriptorMatcher> bfmatcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);

        //Finds the k best matches; first argument is query, second train
        bfmatcher -> knnMatch(currentkf_.descriptor, incoming_.descriptor, knn_matches, 2);
        good_matches_.clear();

        //Lowe's ratio test to filter matches, lower thresh -> more exact
        const float ratio_thresh = 0.4f;

        for (size_t i = 0; i < knn_matches.size(); i++) {
            //checking with distance to 2nd best match
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                good_matches_.push_back(knn_matches[i][0]);
            }
        }
        std::cout << "good matches, size: " << good_matches_.size() << std::endl;
    }



    //sends 2D coordinates of 3D map points in incoming_ frame with their index to Tracking part (observations)
    void FeatureTriangulation::matchToMap() {

        matchedObservations_.id_list.clear();
        matchedObservations_.coord_2d.clear();
        matchedObservations_.t_sec = incoming_.time;

        //get descriptor info of all points in order of their id (order of list)
        //one descriptor per point/per row -> QUERY
        Mat descriptors_3d = getPtsToMatch();

        //to compute 2d observations
        std::vector<DMatch> corresponding = matchDescriptors(descriptors_3d, incoming_.descriptor, 0.4f);

        for (int i = 0; i < corresponding.size(); ++i) {
            matchedObservations_.id_list.push_back(corresponding[i].queryIdx + 1);
            matchedObservations_.coord_2d.push_back(incoming_.keypoints[corresponding[i].trainIdx].pt);
        }

        /*
        //output of entire list of observations
        std::cout << "List of observations: " << std::endl;
        for (int i = 0; i < observations.size(); i++) {
            std::cout << "Point " << i + 1 << ": " << std::endl;
            std::cout << "ID: " << observations[i].id << std::endl;
            std::cout << "2D Coordinates: " << observations[i].coord_2d.x << " " << observations[i].coord_2d.y << std::endl;
        }
         */
    }




    std::vector<Point3f> FeatureTriangulation::triangulateAndStore() {

        std::vector<Point3f> pointcloud;
        std::vector<Point2f> pts_previous;
        std::vector<Point2f> pts_current;
        Mat descriptors_new;

        //write matrix of descriptors of all good matches -> therefore all candidates for triangulation
        Mat descriptors_train_matched;
        for (int i = 0; i < good_matches_.size(); i++) {
            descriptors_train_matched.push_back(currentkf_.descriptor.row(good_matches_[i].trainIdx));
        }

        if (nKeyframes_ == 2) {
            descriptors_new = descriptors_train_matched;

            for (int i = 0; i < good_matches_.size(); i++) {
                pts_previous.push_back(previouskf_.keypoints[good_matches_[i].queryIdx].pt);
                pts_current.push_back(currentkf_.keypoints[good_matches_[i].trainIdx].pt);
            }
        }
        else {
            //filter out only new keypoints (unregistered keypoints)
            descriptors_new = checkIfNew(descriptors_train_matched);
            for (int i = 0; i < new_points_.size(); i++) {
                int s = new_points_[i];
                pts_previous.push_back(previouskf_.keypoints[good_matches_[s].queryIdx].pt);
                pts_current.push_back(currentkf_.keypoints[good_matches_[s].trainIdx].pt);
            }
            std::cout << "pts previous, current size -> to be triangulated: " << std::endl;
            std::cout << pts_previous.size() << std::endl;
            std::cout << pts_current.size() << std::endl;
        }

        //FALLTHROUGH CASE: if no new points found -> exit triangulation process
        if(pts_current.size() < 1) {
            std::vector<Point3f> exception(0);
            return exception;
        }

        //triangulated points (homogeneous coord.), 4xgood_matches_.size()
        Mat coord_homog;
        Mat P1 = previouskf_.proj_mat;
        Mat P2 = currentkf_.proj_mat;

        //OPENCV FUNCTION to triangulate points, ADDS POINT VECTORS (HOM) COLUMNWISE
        triangulatePoints(P1, P2, pts_previous, pts_current, coord_homog);

            //append coordinates to vector for pointcloud generation
            for (int n = 0; n < pts_current.size(); ++n) {

                float x = coord_homog.at<float>(0, n);
                float y = coord_homog.at<float>(1, n);
                float z = coord_homog.at<float>(2, n);
                float w = coord_homog.at<float>(3, n);

                //VECTOR named pointcloud, nth element holds nth point
                pointcloud.push_back(Point3f(x/w, y/w, z/w));
                all_points_ += 1;
            }

            std::vector<Point3f> filtered_pcl = checkBackProjection(pointcloud, pts_current);

        //write descriptor matrix of filtered points
        Mat filtered_descr;
        for (int i = 0; i < indexes_filtered_.size(); i++) {
            filtered_descr.push_back(descriptors_new.row(indexes_filtered_[i]));
        }
            //inserts: landmark coordinates, descriptor of point and nViews
            insertKeyPoints(filtered_descr, filtered_pcl);
            return filtered_pcl;
    }




    std::vector<Point3f> FeatureTriangulation::checkBackProjection(std::vector<Point3f> pcl_unfiltered, std::vector<Point2f> pts_currentkf) {

        std::vector<Point2f> back_projected;
        std::vector<Point3f> pcl_filtered;
        indexes_filtered_.clear();

        Mat transl_vec = (Mat_<double>(3, 1));
        eigen2cv(currentkf_.transl_eigen, transl_vec);
        Mat rot_vec = (Mat_<double>(3, 1));

        Rodrigues(currentkf_.rot_mat, rot_vec);

        Mat cam = (Mat_<double>(3, 3) <<
                                      631.8094467471138, 0.0, 415.26735498345005,
                                      0.0, 631.9958703516292, 300.5214432137359,
                                      0.0, 0.0, 1.0);
        Mat dist_coeff = (Mat_<float>(1, 5) << 0.0,
                                                        0.0,
                                                        0.0,
                                                        0.0,
                                                        0.0);

        projectPoints(pcl_unfiltered, rot_vec, transl_vec, cam, dist_coeff, back_projected);
        assert(pts_currentkf.size() == back_projected.size());

        //filter out points that lie behind camera or have a huge reprojection error
        for (int i = 0; i < pts_currentkf.size(); i++) {
            Point2f temp = pts_currentkf[i] - back_projected[i];
            Mat delta = (Mat_<double>(2, 1) << temp.x, temp.y);
            //check distance (l2 norm of points) and x coordinate (whether "behind camera")
            if ((norm(delta, NORM_L2) < 3.0) && (currentkf_.imu_x > pcl_unfiltered[i].x)) {
                pcl_filtered.push_back(pcl_unfiltered[i]);
                indexes_filtered_.push_back(i);
                filtered_points_ += 1;
            }
        }

        std::cout << "ALL POINTS: " << all_points_ << std::endl;
        std::cout << "FILTERED POINTS: " << filtered_points_ << std::endl;

        return pcl_filtered;
    }




    void FeatureTriangulation::getRotAndProjMat() {


        //Calculate Rotation matrix from quaternion info
        Eigen::Quaterniond q_worldcam(currentkf_.worldToCam.transform.rotation.w,
                                      currentkf_.worldToCam.transform.rotation.x,
                                      currentkf_.worldToCam.transform.rotation.y,
                                      currentkf_.worldToCam.transform.rotation.z);
        q_worldcam.normalize();
        Eigen::Matrix3d R_worldcam = q_worldcam.toRotationMatrix();


        //Eventuell für general callback
//        Eigen::Quaterniond q_worldcam = currentkf_.rotation_eigen;
//        q_worldcam.normalize();
//        Eigen::Matrix3d R_worldcam = q_worldcam.toRotationMatrix();

        eigen2cv(R_worldcam,currentkf_.rot_mat);

        //Concatenate rot_mat and translation vector, RT (3x4)
        Eigen::MatrixXd RT(R_worldcam.rows(), R_worldcam.cols()+currentkf_.transl_eigen.cols());
        RT << R_worldcam, currentkf_.transl_eigen;

        //intrinsic camera parameters, constant
        Eigen::MatrixXd cam(3, 3);
        cam << 631.8094467471138, 0.0, 415.26735498345005,
                0.0, 631.9958703516292, 300.5214432137359,
                0.0, 0.0, 1.0;

        //compute projection matrix (3x4)
        Eigen::MatrixXd P = cam * RT;
        eigen2cv(P,currentkf_.proj_mat);
    }




    std::vector<DMatch> FeatureTriangulation::matchDescriptors(Mat query_descr, Mat train_descr, float thresh) {

        std::vector<std::vector<DMatch> > knn_matches;
        std::vector<DMatch> good_matches;
        Ptr<DescriptorMatcher> bfmatcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);

        //Finds the k best matches for each descriptor from a query set.
        //first argument is query, second train
        bfmatcher->knnMatch(query_descr, train_descr, knn_matches, 2);
        //Lowe's ratio test to filter matches
        const float ratio_thresh = thresh;
        for (size_t i = 0; i < knn_matches.size(); i++) {
            //checking with distance to 2nd best match
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        return good_matches;
    }




//-----------------------------------Transferring functionality of Map class -------------------------------------------
    Mat FeatureTriangulation::getPtsToMatch() {

        Mat complete_descriptorMat;
        for (int i = 0; i < map_.registeredPoints_.size(); i++) {
            complete_descriptorMat.push_back(map_.registeredPoints_[i].pt_descriptor);
        }
        return complete_descriptorMat;

    }


//return descriptors of new keypoints
    Mat FeatureTriangulation::checkIfNew(Mat initial_matches) {

        new_points_.clear();

        //join descriptor data back to form one matrix, with descriptor always at entry of keypoint ID
        Mat complete_descriptorMat = getPtsToMatch();

        //compare incoming descriptors with complete descriptor matrix and choose only new ones
        std::vector<DMatch> points_alreadyPresent = matchDescriptors(complete_descriptorMat, initial_matches, 0.4f);
        std::cout << "Number of keypoints being checked: " << initial_matches.size().height << std::endl;
        std::cout << "Number of keypoints already present: " << points_alreadyPresent.size() << std::endl;

        //indexes of points already present in Keypoint list
        /*
        std::cout << "train image index, points already present: " << std::endl;
        for (size_t i = 0; i < points_alreadyPresent.size(); i++) {
            std::cout << points_alreadyPresent[i].trainIdx << std::endl;
        }
         */

        //list of indexes of initial matches
        std::vector<int> index_list;
        for (int i = 0; i < initial_matches.size().height; i++) {
            index_list.push_back(i);
        }

        //numbers in new_point vector are rows where descriptor was taken from
        new_points_ = findNewKeypts(points_alreadyPresent, index_list);
        std::cout << "Number of keypoints to be added: " << new_points_.size() << std::endl;

        Mat descriptors_new;
        for (int i = 0; i < new_points_.size(); i++) {
            descriptors_new.push_back(initial_matches.row(new_points_[i]));
        }

        return descriptors_new;
    }



    //register new points with all of its info (ID, 3D, descriptor, nViews) in list
    void FeatureTriangulation::insertKeyPoints(Mat matched_descr, std::vector<Point3f> pts_filtered) {
        //called after triangulation and filtering
        assert(matched_descr.size().height == pts_filtered.size());
        n_ = map_.registeredPoints_.size();

        for (int i = n_; i < (n_ + pts_filtered.size()); i++) {
            //indexes?
            Eigen::Vector3d coord(pts_filtered[i-n_].x, pts_filtered[i-n_].y, pts_filtered[i-n_].z);
            target_tracking::Map::PointData new_point((i+1), coord);
            //new_point.landmark.printData();
            new_point.n_views = 1;
            new_point.pt_descriptor = matched_descr.row(i-n_);

            map_.registeredPoints_.push_back(new_point);
        }

        std::cout << "Size of MAP DATA vector after adding new points: " << map_.registeredPoints_.size() << std::endl;
    }



    //return only keypoints that have not been registered yet
    std::vector<int> FeatureTriangulation::findNewKeypts(std::vector<DMatch> already_present, std::vector<int> indexes_initial) {

        std::unordered_set<int> pts_present;
        std::vector<int> missing_points;

        //fill hash table
        for (int i = 0; i < already_present.size(); i++) {
            pts_present.insert(already_present[i].trainIdx);
        }

        //find all elements of index_list that are not present in pts_present
        for (int i = 0; i < indexes_initial.size(); i++) {
            if (pts_present.find(indexes_initial[i]) == pts_present.end()) {
                missing_points.push_back(indexes_initial[i]);
            }
        }
        return missing_points;
    }




//----------------------------------------------------------------------------------------------------------------------






} /* namespace */
