#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "util.hpp"
#include "carmaker_tracking/pointInformationarray.h"
#include "carmaker_tracking/pointInformation.h"
#include "carmaker_tracking/Front_vehicle_state.h"
#include "string.h"
#include "std_msgs/String.h"

// #include "kalman_hand.hpp"

ros::Publisher pub, pub2, pub3, pub_vis, pub_test, pub_information, pub_front_vehicle_information, emgergency_stop_pub;

// double cluster_value, centroid_distance, leaf_size;
double cluster_value = 0.5;
double centroid_distance = 1.6;
double leaf_size = 0.3;


class frameTracker
{

public:

    // std::vector<std::vector<pcl::PointXYZI>> compareVector;
    pcl::PointCloud<pcl::PointXYZI> outCloud;
    lidarUtil util;
    std::string limit_signal;


    void signalCallback(const std_msgs::String::ConstPtr& msg) {

        limit_signal = msg->data;
//		std::cout<<"msg: "<<limit_signal<<std::endl;
    }

    void drawMarker(pcl::PointCloud<pcl::PointXYZI>& cloud , pcl::PointXYZI& centroid, double distance, int id) {

        Eigen::Vector4f center;
        Eigen::Vector4f min;
        Eigen::Vector4f max;

        center << centroid.x, centroid.y, 0, 0;        
        pcl::getMinMax3D(cloud, min, max);
        // std::cout<<"min"<<min<<"Max:"<<max<<std::endl;
        pub_vis.publish(util.mark_centroid(pcl_conversions::fromPCL(cloud.header), center, min, max, "velodyne", distance, id, 0, 255, 0));
        
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>> checkCluster(std::vector<pcl::PointCloud<pcl::PointXYZI>>& vec) {

        std::vector<pcl::PointCloud<pcl::PointXYZI>> outVector;
        std::vector<pcl::PointCloud<pcl::PointXYZI>>::iterator it;

        for(int i=0; i < vec.size(); i++) {

            Eigen::Vector4f min;
            Eigen::Vector4f max;
            pcl::getMinMax3D(vec.at(i), min, max);

            double length = max[1] - min[1];
            double real_length = sqrt(pow(length, 2));
//            std::cout<<"Length: "<< real_length<<std::endl;
            if(real_length < 3) {

                outVector.push_back(vec.at(i));
            } 
        }

        return outVector;
    }


    void publishClusterInformation(pcl::PointXYZI& centroid_, float distance_) {

        carmaker_tracking::pointInformation data;
        carmaker_tracking::pointInformationarray msg;

        data.x = centroid_.x, data.y = centroid_.y, data.intensity = centroid_.intensity, data.distance = distance_;
        msg.points.push_back(data);

        if(!((-4.3 < data.x && data.x <0) && (-0.1<data.y && data.y <0.1))) {

            if(limit_signal == "left") {

                if(data.y < 2.0) {

                    pub_information.publish(msg);
                }
            }
            else if(limit_signal == "right") {

                if(data.y > -2.0) {

                    pub_information.publish(msg);
                }
            }
            else if(limit_signal == "no") {

                pub_information.publish(msg);
            }
        }

    }


    void verifyState(pcl::PointXYZI& centroid, float distance_) {

        carmaker_tracking::emergency_state emergency_msg;
        carmaker_tracking::Front_vehicle_state msg;

        if( centroid.y <= 2 & centroid.y >= -2
                & centroid.x > 0 & centroid.x < 50) {
            // std::cout<<"Vehicle in front"<<std::endl;

            msg.x = centroid.x, msg.y = centroid.y, msg.distance = distance_, msg.intensity = centroid.intensity;
            util.waittTimeReset();

            if(distance_ > 50) {
                //  Normal mode
                emergency_msg.state = 0;
            }
            else if((distance_ > 25) & (distance_ < 50)) {
                // Alert
                emergency_msg.state = 1;
                util.initCount();

            }
            else if((distance_ > 0 ) & (distance_ < 25)) {
                //  emergency braking
                emergency_msg.state = 2;
                // util.delay = util.delay + 1;
                // std::cout<<"delay"<<util.delay<<std::endl;
                util.dealyCount();
                if(util.delay == 150) {
                    util.initCount();
                    std::cout<<"Go!"<<std::endl;
                    emergency_msg.state = 3;
                }

            }
            
            pub_front_vehicle_information.publish(msg);
            emgergency_stop_pub.publish(emergency_msg);
        }
        else {
            // std::cout<<"without"<<std::endl;
            util.waitTime(emergency_msg, emgergency_stop_pub);
        }


    }

    
    void Callback(const sensor_msgs::PointCloud2& msg)
    {

        //set variable and convert ros msg

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr projection_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(msg, *cloud);


        //Point roi & Point projection

        output_cloud = util.point_roi(cloud);
        projection_cloud = util.point_projection(output_cloud);



            // std::cout<<"Input: "<<cloud->points.size()<<" ( "<<pcl::getFieldsList(*cloud)<<")"<<std::endl;
        //  voxel
        
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(projection_cloud);
        sor.setLeafSize(0.3f,0.3f,0.3f);
        sor.setLeafSize(leaf_size,leaf_size,leaf_size);
        sor.filter(*projection_cloud);

        pcl::PCLPointCloud2 test;
        pcl::toPCLPointCloud2(*projection_cloud, test);
        sensor_msgs::PointCloud2 test_minmax; 
        pcl_conversions::fromPCL(test, test_minmax);
        test_minmax.header.frame_id = "VLP16_Front";
        pub_test.publish(test_minmax); 

        // std::cout << "Output : " << cloud_filtered->points.size () << " (" << pcl::getFieldsList (*cloud_filtered) <<")"<< std::endl;

        //Create  object
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(projection_cloud);

        // std::vector<pcl::PointIndices> cluster_indices;
        // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        // ec.setClusterTolerance(cluster_value);
        // ec.setMinClusterSize(5);
        // ec.setMaxClusterSize(300);
        // ec.setSearchMethod(tree);
        // ec.setInputCloud(projection_cloud);
        // ec.extract(cluster_indices);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_value);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(300);
        ec.setSearchMethod(tree);
        ec.setInputCloud(projection_cloud);
        ec.extract(cluster_indices);


        std::vector<pcl::PointCloud<pcl::PointXYZI>> TotalCloud; 
        std::vector<pcl::PointCloud<pcl::PointXYZI>> preTotalCloud; 
        pcl::PointCloud<pcl::PointXYZI> final_cloud; 

        for( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZI> tempcloud; 

            for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                pcl::PointXYZI pt = projection_cloud->points[*pit];
                pcl::PointXYZI pt2;

                pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                // pt2.intensity = (float)(j+1);

                tempcloud.push_back(pt2);
                final_cloud.push_back(pt2);
 
            }

            TotalCloud.push_back(tempcloud);

        }

        preTotalCloud = checkCluster(TotalCloud);
        std::vector<float> intensityVector;
        intensityVector = util.selectNumber(preTotalCloud, centroid_distance);
        util.extractDistance();


        pcl::PCLPointCloud2 cloud_clustered;
        pcl::toPCLPointCloud2(util.paintColor(preTotalCloud, intensityVector), cloud_clustered);
        sensor_msgs::PointCloud2 output_clustered; 
        pcl_conversions::fromPCL(cloud_clustered, output_clustered);
        // output_clustered.header.frame_id = "velodyne";
        output_clustered.header.frame_id = "VLP16_Front";
        pub.publish(output_clustered);
        final_cloud.clear();


        pcl::PCLPointCloud2 center_cloud;
        pcl::toPCLPointCloud2(util.outCloud, center_cloud);
        sensor_msgs::PointCloud2 center; 
        pcl_conversions::fromPCL(center_cloud, center);
        // center.header.frame_id = "velodyne";
        center.header.frame_id = "VLP16_Front";
        pub2.publish(center); 
        util.outCloud.clear();

        pcl::PCLPointCloud2 kalman_predict;
        pcl::toPCLPointCloud2(util.outKalman, kalman_predict);
        sensor_msgs::PointCloud2 kalman_center; 
        pcl_conversions::fromPCL(kalman_predict, kalman_center);
        // kalman_center.header.frame_id = "velodyne";
        kalman_center.header.frame_id = "VLP16_Front";
        pub3.publish(kalman_center); 

        pcl::PointCloud<pcl::PointXYZI> minmax_test;
        for (int i =0 ; i <util.outKalman.size(); i++) {
            
            Eigen::Vector4f min;
            Eigen::Vector4f max;
            // pcl::PointXYZI test_point1;
            // pcl::PointXYZI test_point2;

            drawMarker(util.outMinMax.at(i), util.outKalman.at(i), util.distanceVector.at(i), i);
            publishClusterInformation(util.outKalman.at(i), util.distanceVector.at(i));
            // pcl::getMinMax3D(util.outMinMax.at(i), min, max);
            // test_point1.x = 0, test_point1.y = 0, test_point1.z = 0;
            // test_point2.x = 2, test_point2.y = -1, test_point2.z = 0;
        }


        //for start alert

        for (int i =0 ; i <util.outKalman.size(); i++) {
            
            Eigen::Vector4f min;
            Eigen::Vector4f max;
            // pcl::PointXYZI test_point1;
            // pcl::PointXYZI test_point2;

            drawMarker(util.outMinMax.at(i), util.outKalman.at(i), util.distanceVector.at(i), i);
            verifyState(util.outKalman.at(i), util.distanceVector.at(i));

        }



        util.outKalman.clear();
        util.distanceVector.clear();

        // pcl::PCLPointCloud2 test;
        // pcl::toPCLPointCloud2(minmax_test, test);
        // sensor_msgs::PointCloud2 test_minmax; 
        // pcl_conversions::fromPCL(test, test_minmax);
        // test_minmax.header.frame_id = "velodyne";
        // pub_test.publish(test_minmax); 
        // minmax_test.clear();

    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv,"carmaker_tracking");
    ros::NodeHandle nh;

    nh.getParam("cluster_value_",cluster_value);
    nh.getParam("centroid_distance_",centroid_distance);
    nh.getParam("voxel_leaf_size_", leaf_size);

    frameTracker tracker;

    ros::Subscriber signal_sub = nh.subscribe("/yellow_lane",1, &frameTracker::signalCallback, &tracker);
    ros::Subscriber sub = nh.subscribe("/velodyne_points",1, &frameTracker::Callback, &tracker);

    pub = nh.advertise<sensor_msgs::PointCloud2>("tracking", 1);

    pub2 = nh.advertise<sensor_msgs::PointCloud2>("center", 1);

    pub3 = nh.advertise<sensor_msgs::PointCloud2>("Kalman_predict", 1);

    pub_vis = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 1);

    pub_test = nh.advertise<sensor_msgs::PointCloud2>("test", 1);

    pub_information = nh.advertise<carmaker_tracking::pointInformationarray>("axis_distance_msg", 1);

    pub_front_vehicle_information = nh.advertise<carmaker_tracking::Front_vehicle_state>("Front_vehicle_information", 1);

    emgergency_stop_pub = nh.advertise<carmaker_tracking::emergency_state>("Emergency_state", 1);

    ros::spin();

}
