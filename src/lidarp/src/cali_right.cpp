#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#define VPoint velodyne_pointcloud::PointXYZIR
#define Point2 pcl::PointXYZI

using namespace std;

ros::Publisher pub1;


void input(const sensor_msgs::PointCloud2ConstPtr& scan)
{

    // Msg to pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*scan,*cloud); // ros msg 에서 pcl cloud 데이터로 변환

    //회전변환행렬
//	Eigen::MatrixXd transform_1 = Eigen::MatrixXd::Identity();
    Eigen::Matrix4f transform_1 ;
    transform_1 << -0.00549059,    0.999942 , 0.00922014 ,  -3.676,
                -0.999973, -0.00553455,  0.00474851 ,   -1.03158,
                0.00479927, -0.00919383  ,  0.999946  ,  0.233184,
                0    ,       0      ,     0   ,        1;

/*    transform_1 << 0.686155, -0.706304, 0.0253142, 0.04338,
    				0.706604, 0.685845, 0.0148543, 0.94482,
    				0.00787726,0.0282738,0.998896,0.25208,
    				0,0,0,1;*/

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZI> );
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1);

    pcl::PassThrough<pcl::PointXYZI> pass;

    pass.setInputCloud(transformed_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.2,20);
    pass.filter(*out_cloud);

    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(*out_cloud, cloud_p); 

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);
    
    pub1.publish(output);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "input_r");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("pointcloud/vlp_right", 100, input); //front ouster
	pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/right_velodyne_points_rotated", 100);
	ros::spin();
}
