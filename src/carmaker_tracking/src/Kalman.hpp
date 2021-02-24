#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <eigen3/Eigen/Dense>

class kalmanFilter {

private:

    Eigen::Vector4f stateVector;
    Eigen::Matrix4f covMatrix;
    Eigen::MatrixXf H_matrix;
    Eigen::Matrix4f predictionMatrix;
    Eigen::Matrix4f Q_matrix;


    kalmanFilter() {

        float dt = 0.1;

        stateVector << 0, 0, 0.2, 0;

        predictionMatrix << 1,0,T,0,
                            0,1,0,T,
                            0,0,1,0,
                            0,0,0,1;

        H_matrix << 1,0,0,0,
                    0,1,0,0;
        
        Q_matrix << Eigen::Matrix4f::Identity() * 0.001;
        
        covMatrix = Eigen::Matrix4f::Identity();
        

        
    }


    void predict() {

        Eigen::Vector4f pre_mean;
        Eigen::Matrix4f pre_cov;

        pre_mean = predictionMatrix * stateVector;
        pre_cov = predictionMatrix*covMatrix*predictionMatrix.transpose() + Q_matrix;

        


    }

};



