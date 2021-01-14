#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>    
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>



ros::Publisher pub;

void points_callback_front(const sensor_msgs::PointCloud msg){
  
	sensor_msgs::PointCloud2 point2;
	sensor_msgs::convertPointCloudToPointCloud2(msg, point2);
	point2.is_dense = 1;
	pub.publish(point2);    
//	std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"VLP_Front");
//    ros::init(argc,argv,"VLP_Left");
    ros::NodeHandle nh;
//	ros::NodeHandle p_nh("~");

    ros::Subscriber sub;
    sub = nh.subscribe<sensor_msgs::PointCloud>("/pointcloud/vlp_front",1,points_callback_front);
    pub = nh.advertise<sensor_msgs::PointCloud2>("points_front",1);
//    sub = nh.subscribe<sensor_msgs::PointCloud>("/pointcloud/vlp_front",1,points_callback_front);
//	pub = nh.advertise<sensor_msgs::PointCloud2>("points_front",1);
    ros::spin();

    return 0;
}

//pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
//  {
//    pcl::PointCloud<pcl::PointXYZ> cloud_dst;
//    pcl::fromROSMsg(cloudmsg, cloud_dst);
//    return cloud_dst;
//  }
