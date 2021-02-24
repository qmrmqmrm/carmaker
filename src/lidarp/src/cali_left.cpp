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

void input(const sensor_msgs::PointCloud2::ConstPtr &scan)
{

    // Msg to pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*scan,*cloud); // ros msg 에서 pcl cloud 데이터로 변환

    //회전변환행렬
//	Eigen::MatrixXd transform_1 = Eigen::MatrixXd::Identity();
    Eigen::Matrix4f transform_1 ;
        transform_1 << 0.00714647,   -0.999959 , 0.00560343 ,   -3.72064,
                    0.999974 , 0.00715156, 0.000888723  ,  0.782616,
                    -0.00092876,  0.00559694 ,   0.999984 ,  0.091374,
                        0   ,        0    ,       0     ,      1;

/*    transform_1 << 0.723518, 0.680757, -0.0462027, -0.79437,
    				-0.681641, 0.724179, 0.00851417, 0.886662,
    				0.0208478,0.0421016,0.998896,0.183407,
    				0,0,0,1;*/

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> );
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZI> );

    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1);
//	pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);

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
	ros::init(argc, argv, "input");
	ros::NodeHandle nh;
	pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/left_velodyne_points_rotated", 10);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/pointcloud/vlp_left", 10, input); //front ouster
//	pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/left_velodyne_points_rotated", 10);
	ros::spin();
}
