#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/PointCloud2.h>
#include "carmaker_tracking/pointInformationarray.h"
#include "carmaker_tracking/pointInformation.h"
#include "carmaker_tracking/Front_vehicle_state.h"
#include "carmaker_tracking/emergency_state.h"
#include "carmaker_tracking/Tolocal.h"

ros::Publisher pub;

class localFlag {

public:
    bool lock = 0;
    float target_intensity = 0;

    void callback1( const carmaker_tracking::emergency_state& msg) {

        std::cout<<"msg: "<<msg.state<<std::endl;
        if( msg.state == 3) {
            std::cout<<"Unlocked!!!!"<<std::endl;
            lock = 1;
        }
    }

    void callback2( const carmaker_tracking::Front_vehicle_state& msg) {
        
        if(lock == 1) {

            target_intensity = msg.intensity;
        }
    }
            

    void callback3( const carmaker_tracking::pointInformationarray& msg) {

        if(lock == 1) {
            
            carmaker_tracking::Tolocal local_msg;

            for( int i =0; i < msg.points.size(); i++) {

                if(msg.points[i].intensity == target_intensity) {
                    
                    if( msg.points[i].x < -2.5) {
                        
                        std::cout<<"local path end!"<<std::endl;
                        local_msg.state = 1;
                        lock = 0;
                        pub.publish(local_msg);
                    }
                    else {
                        local_msg.state = 0;
                        pub.publish(local_msg);
                    }

                }

            }
        }

    }


};


int main(int argc, char** argv) {

    ros::init(argc, argv, "flag_to_local");
    ros::NodeHandle nh;
    localFlag LF;
    std::cout<<"start"<<std::endl;

    ros::Subscriber sub = nh.subscribe("/Emergency_state", 1, &localFlag::callback1, &LF);
    ros::Subscriber sub2 = nh.subscribe("/Front_vehicle_information", 1, &localFlag::callback2, &LF);
    ros::Subscriber sub3 = nh.subscribe("/axis_distance_msg",1, &localFlag::callback3, &LF);

    pub = nh.advertise<carmaker_tracking::Tolocal>("Local_state", 1);

    ros::spin();

}