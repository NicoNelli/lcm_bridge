#include "lcm/lcm-cpp.hpp"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "lcm_messages/geometry/pose.hpp"
#include "nav_msgs/Odometry.h"
#include "utils/TimeHelpers.hpp"

#include <queue>

lcm::LCM handler;
geometry::pose temp_plat;
geometry::pose temp_plat_prev;
TimeManager tm;
bool first = true;
int window = 6;
ros::Publisher pub;
typedef std::queue<double> queue_t;

queue_t stack_x;
queue_t stack_y;
queue_t stack_z;

double integral_x = 0;
double integral_y = 0;
double integral_z = 0;

void mean(double& sum,double plus,double minus){

    sum += plus - minus;

}

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

        double vx = dx/tm._dt;
        double vy = dy/tm._dt;
        double vz = dz/tm._dt;

        //Mean for smoothing
        if (stack_x.size() == window) {
            mean(integral_x,vx,stack_x.front());
            stack_x.pop();
        } else
            mean(integral_x,vx,0);

        if (stack_y.size() == window) {
            mean(integral_y, vy, stack_y.front());
            stack_y.pop();
        } else
            mean(integral_y,vy,0);

        if (stack_z.size() == window) {
            mean(integral_z, vz, stack_z.front());
            stack_z.pop();
        } else
            mean(integral_z,vz,0);

        stack_x.push(vx);
        stack_y.push(vy);
        stack_z.push(vz);

        //Something has been pushed, size is not 0
        temp_plat.velocity[0] = integral_x/stack_x.size();
        temp_plat.velocity[1] = integral_y/stack_y.size();
        temp_plat.velocity[2] = integral_z/stack_z.size();

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
    
    //ROS Helper for visualization
    nav_msgs::Odometry temp_pos;
    temp_pos.pose.pose.position.x = temp_plat.position[0];
    temp_pos.pose.pose.position.y = temp_plat.position[1];
    temp_pos.pose.pose.position.z = temp_plat.position[2];

    temp_pos.twist.twist.linear.x = temp_plat.velocity[0];
    temp_pos.twist.twist.linear.y = temp_plat.velocity[1];
    temp_pos.twist.twist.linear.z = temp_plat.velocity[2];

    pub.publish(temp_pos); 

}

int main(int argc, char **argv)
{

    //ROS helpers
    ros::init(argc, argv, "ros2lcm_plat");
    ros::NodeHandle n;
    ros::Subscriber plat_sub = n.subscribe("/PlatformPose",1,&posePCB);
    pub = n.advertise<nav_msgs::Odometry>("/PlatformPose_filtered", 1);
    tm.updateTimer();
    //main loop

    ros::spin();

    return 0;

}
