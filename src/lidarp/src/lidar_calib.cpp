#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
	
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
	
#include <math.h>
	
using namespace message_filters;
using namespace sensor_msgs;
	
ros::Publisher Cal_F_pc_pub;
ros::Publisher Cal_R_pc_pub;
ros::Publisher Cal_L_pc_pub;
ros::Publisher Combine_pc_pub;

	
pcl::PointCloud<pcl::PointXYZI> x_coordinate_filter(const pcl::PointCloud<pcl::PointXYZI> origin_pc) {
	pcl::PointCloud<pcl::PointXYZI> x_filtered;
	x_filtered.header.frame_id = "/VLP16_Front";
		
	int cloudSize = origin_pc.points.size();
	for (int i = 0 ; i < cloudSize ; i++) {
	if(!std::isnan (origin_pc.points[i].x) && !std::isnan (origin_pc.points[i].y) && !std::isnan (origin_pc.points[i].z) && origin_pc.points[i].x > 0.1) { // point.x > 0 only
	x_filtered.push_back(origin_pc.points[i]);
	}
	}
		
	return x_filtered;
}
	
pcl::PointCloud<pcl::PointXYZI> pc_combine(const pcl::PointCloud<pcl::PointXYZI> Front, const pcl::PointCloud<pcl::PointXYZI> Right, const pcl::PointCloud<pcl::PointXYZI> Left) {
	pcl::PointCloud<pcl::PointXYZI> combined_point;
	combined_point.header.frame_id = "/VLP16_Front";
		
	int cloudSize = Front.points.size();
	for (int i = 0 ; i < cloudSize ; i++) {
		if(!std::isnan (Front.points[i].x) && !std::isnan (Front.points[i].y) && !std::isnan (Front.points[i].z)) {
		combined_point.push_back(Front.points[i]);
		}
	}
		
	cloudSize = Right.points.size();
	for (int i = 0 ; i < cloudSize ; i++) {
		if(!std::isnan (Right.points[i].x) && !std::isnan (Right.points[i].y) && !std::isnan (Right.points[i].z)) {
		combined_point.push_back(Right.points[i]);
		}
	}
		
	cloudSize = Left.points.size();
	for (int i = 0 ; i < cloudSize ; i++) {
		if(!std::isnan (Left.points[i].x) && !std::isnan (Left.points[i].y) && !std::isnan (Left.points[i].z)) {
		combined_point.push_back(Left.points[i]);
		}
	}
		
	return combined_point;
}
	
void callback(const PointCloud2ConstPtr& F_pc, const PointCloud2ConstPtr& R_pc, const PointCloud2ConstPtr& L_pc) {

	pcl::PointCloud<pcl::PointXYZI> F_cal;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pre_F_cal(new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PointCloud<pcl::PointXYZI> R_cal;
	pcl::PointCloud<pcl::PointXYZI> L_cal;
		
	pcl::fromROSMsg(*F_pc, *pre_F_cal);
	pcl::fromROSMsg(*R_pc, R_cal);
	pcl::fromROSMsg(*L_pc, L_cal);

    pcl::PassThrough<pcl::PointXYZI> pass;

	pass.setInputCloud(pre_F_cal);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.2,3);
    pass.filter(*pre_F_cal);
	pass.setInputCloud(pre_F_cal);
	pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.1,500);
    pass.filter(F_cal);
	

	// Point cloud combine
	pcl::PointCloud<pcl::PointXYZI> combine_point_cloud;
	combine_point_cloud = pc_combine(F_cal, R_cal, L_cal);

	sensor_msgs::PointCloud2 Comb;
	pcl::toROSMsg(combine_point_cloud, Comb);
	Comb.header.stamp = L_pc->header.stamp;
	Combine_pc_pub.publish (Comb);
}
	
int main(int argc, char** argv) {
	// Initialize ROS
	ros::init (argc, argv, "velo_cal");
	ros::NodeHandle nh;
		
	// Create a ROS publisher for the output point cloud
	Cal_F_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("Calib_Front_velo", 1);
	Cal_R_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("Calib_Right_velo", 1);
	Cal_L_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("Calib_Left_velo", 1);
	Combine_pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points", 1);
		
	// Create a ROS subscriber for the input point cloud
	Subscriber<PointCloud2> F_pc_sub(nh, "/pointcloud/vlp_front", 1);
	Subscriber<PointCloud2> R_pc_sub(nh, "/right_velodyne_points_rotated", 1);
	Subscriber<PointCloud2> L_pc_sub(nh, "/left_velodyne_points_rotated", 1);
		
	typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2> SyncPolicy;
	Synchronizer<SyncPolicy> sync(SyncPolicy(10), F_pc_sub, R_pc_sub, L_pc_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3));

	ros::spin ();
} 	 


