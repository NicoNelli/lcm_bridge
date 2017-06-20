#include "lcm/lcm-cpp.hpp"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "lcm_messages/geometry/pose.hpp"
#include "utils/TimeHelpers.hpp"

lcm::LCM handler;
geometry::pose temp_plat;
geometry::pose temp_plat_prev;
TimeManager tm;
bool first = true;

void calculateVel(){

    if(first){
        temp_plat.velocity[0] = 0;
        temp_plat.velocity[1] = 0;
        temp_plat.velocity[2] = 0;
        first = false;
    }else{

        //Calculate platform velocity here
        double dx = temp_plat.position[0] - temp_plat_prev.position[0];
        double dy = temp_plat.position[1] - temp_plat_prev.position[1];
        double dz = temp_plat.position[2] - temp_plat_prev.position[2];

        temp_plat.velocity[0] = dx/tm._dt;
        temp_plat.velocity[1] = dy/tm._dt;
        temp_plat.velocity[2] = dz/tm._dt;

    }

    temp_plat_prev.position[0] = temp_plat.position[0];
    temp_plat_prev.position[1] = temp_plat.position[1];
    temp_plat_prev.position[2] = temp_plat.position[2];

}

void posePCB(geometry_msgs::PoseStamped msg){

    tm.updateTimer();
    temp_plat.position[0] = msg.pose.position.x;
    temp_plat.position[1] = msg.pose.position.y;
    temp_plat.position[2] = msg.pose.position.z;

    calculateVel();

    handler.publish("platform/pose",&temp_plat);

    std::cout << "Platform pose" << std::endl;
    std::cout << temp_plat.position[0] << std::endl;
    std::cout << temp_plat.position[1] << std::endl;
    std::cout << temp_plat.position[2] << std::endl;
    std::cout << "Platform velo" << std::endl;
    std::cout << temp_plat.velocity[0] << std::endl;
    std::cout << temp_plat.velocity[1] << std::endl;
    std::cout << temp_plat.velocity[2] << std::endl;

}

int main(int argc, char **argv)
{

    //ROS helpers
    ros::init(argc, argv, "ros2lcm_plat");
    ros::NodeHandle n;
    ros::Subscriber plat_sub = n.subscribe("/PlatformPose",1,&posePCB);

    tm.updateTimer();
    //main loop

    ros::spin();

    return 0;

}
