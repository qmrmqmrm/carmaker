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

void points_callback_left(const sensor_msgs::PointCloud msg){
  
	sensor_msgs::PointCloud2 point2;
	sensor_msgs::convertPointCloudToPointCloud2(msg, point2); 
	point2.is_dense = 1;
	pub.publish(point2);    
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"VLP_Left");
//    ros::init(argc,argv,"VLP_Left");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    ros::Subscriber sub;
    sub = nh.subscribe<sensor_msgs::PointCloud>("/pointcloud/vlp_left",1,points_callback_left);
    pub = nh.advertise<sensor_msgs::PointCloud2>("points_left",1);
    ros::spin();

    return 0;
}

