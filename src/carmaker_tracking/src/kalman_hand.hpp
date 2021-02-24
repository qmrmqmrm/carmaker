#ifndef KALMAN_HAND_HPP
#define KALMAN_HAND_HPP

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <eigen3/Eigen/Dense>
#include <pcl-1.8/pcl/point_types.h>
// #include "util.hpp"

class KalmanFilter_hand {

public:

    std::vector<pcl::PointXYZI> preMean;
    float T = 0.1;

    KalmanFilter_hand() { }

    ~KalmanFilter_hand() { }


    void predict(const pcl::PointXYZI& pre_state, const pcl::PointXYZI& cur_state) {

        cv::KalmanFilter KFT( 4,2,0);
        // cv::KalmanFilter KF( 4,2,0);
        KFT.transitionMatrix = (cv::Mat_<float>(4,4) << 1,0 , T, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1);
        // preMean = centroid;
        KFT.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
        KFT.measurementMatrix.at<float>(0) = 1.0f;
        KFT.measurementMatrix.at<float>(5) = 1.0f;        

        cv::setIdentity(KFT.processNoiseCov, cv::Scalar::all(1e-4));
        cv::setIdentity(KFT.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(KFT.errorCovPost, cv::Scalar::all(5));

        KFT.statePre.at<float>(0) = pre_state.x;
        KFT.statePre.at<float>(1) = pre_state.y;
        KFT.statePre.at<float>(2) = 0;
        KFT.statePre.at<float>(3) = 0;

        cv::Mat_<float> measure(4,1); 
        measure << cur_state.x, cur_state.y, 0, 0;
        cv::Mat prediction = KFT.predict();
        cv::Mat estimated = KFT.correct(measure);
        std::cout<<estimated<<std::endl;


    }





};

#endif