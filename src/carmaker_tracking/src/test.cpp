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

// #include "kalman_hand.hpp"

ros::Publisher pub;

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

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_roi(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl_conversions::toPCL(msgs, cloud);
        pcl::PassThrough<pcl::PointXYZI> pass;
        // pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(point);    
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-6, 6);
        pass.filter(*point);
        pass.setInputCloud(point);   
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.2,1);
        pass.filter(*point);
        pass.setInputCloud(point);   
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-70,1000);
        pass.filter(*out_cloud);

        return out_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_projection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point)
    {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

        coefficients->values.resize(4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1.0;
        coefficients->values[3] = 0;
        
        pcl::ProjectInliers<pcl::PointXYZI> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(point);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);

        return cloud_projected;    
    }



    
    void Callback(const sensor_msgs::PointCloud2& msg)
    {

        //set variable and convert ros msg

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr projection_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(msg, *output_cloud);

        output_cloud = point_roi(output_cloud);
        projection_cloud = point_projection(output_cloud);

        
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(projection_cloud);
        sor.setLeafSize(0.3f,0.3f,0.3f);
        sor.setLeafSize(leaf_size,leaf_size,leaf_size);
        sor.filter(*projection_cloud);


        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(projection_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_value);
        ec.setMinClusterSize(27);
        ec.setMaxClusterSize(300);
        ec.setSearchMethod(tree);
        ec.setInputCloud(projection_cloud);
        ec.extract(cluster_indices);



        std::vector<pcl::PointCloud<pcl::PointXYZI>> TotalCloud; 
        pcl::PointCloud<pcl::PointXYZI> final_cloud; 
        int j = 0;
        int init = 0;

        for( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {

            // if(it.indices[0] <2000) {
            int total_point = 0;
            for(int k = 0; k < cluster_indices[init].indices.size(); k++) {

                // std::cout<<"number: "<<cluster_indices[init].indices[k]<<std::endl;
                // std::cout<<"더해지는 값"<<cluster_indices[init].indices[k]<<std::endl;
                total_point += cluster_indices[init].indices[k];
            }
            int result = total_point / cluster_indices[init].indices.size();
            std::cout<<"total 평균: "<<result<<std::endl;

            if(result < 130){

                // std::cout<<"Cluster Number: "<< cluster_indices[init].indices[0]<<" "<<cluster_indices[init].indices.size()<<std::endl;
                for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                {
                    pcl::PointXYZI pt = projection_cloud->points[*pit];
                    pcl::PointXYZI pt2;

                    pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                    pt2.intensity = (float)(j+1);

                    // tempcloud.push_back(pt2);
                    final_cloud.push_back(pt2);
                    j += 1;
    
                }

            }

            init += 1;

        


            // }


        }



        pcl::PCLPointCloud2 cloud_clustered;
        pcl::toPCLPointCloud2(final_cloud, cloud_clustered);
        sensor_msgs::PointCloud2 output_clustered; 
        pcl_conversions::fromPCL(cloud_clustered, output_clustered);
        // output_clustered.header.frame_id = "velodyne";
        output_clustered.header.frame_id = "VLP16_Front";
        pub.publish(output_clustered);
        final_cloud.clear();



    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv,"carmaker_test");
    ros::NodeHandle nh;

    nh.getParam("cluster_value_",cluster_value);
    nh.getParam("centroid_distance_",centroid_distance);
    nh.getParam("voxel_leaf_size_", leaf_size);

    frameTracker tracker;
    ros::Subscriber sub = nh.subscribe("/velodyne_points",1, &frameTracker::Callback, &tracker);

    pub = nh.advertise<sensor_msgs::PointCloud2>("clustered_points", 1);

    ros::spin();

}