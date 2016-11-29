#include "lcm/lcm-cpp.hpp"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "lcm_messages/geometry/pose.hpp"
#include "common/MavState.h"
#include "common/CallbackHandler.hpp"
#include <poll.h>

lcm::LCM handler;
geometry::pose lcm_pose;
CallbackHandler call;

void odometryCallback(geometry_msgs::PoseStamped pose){

    MavState s;
    s.setPosition((float)pose.pose.position.x,(float)pose.pose.position.y,(float)pose.pose.position.z);
    s.setOrientation((float)pose.pose.orientation.w,(float)pose.pose.orientation.x,(float)pose.pose.orientation.y,(float)pose.pose.orientation.z);
    float roll, pitch, yaw;


    lcm_pose.position[0] = pose.pose.position.x;
    lcm_pose.position[1] = pose.pose.position.y;
    lcm_pose.position[2] = -pose.pose.position.z;

    lcm_pose.orientation[0] = pose.pose.orientation.w;
    lcm_pose.orientation[1] = pose.pose.orientation.x;
    lcm_pose.orientation[2] = pose.pose.orientation.y;
    lcm_pose.orientation[3] = pose.pose.orientation.z;
/*
    s.orientation(&roll,&pitch,&yaw);
    std::cout << "************************************************"<< std::endl;
    std::cout << "Suspected pose POSITION: " << s.getX()<<" " << s.getY() << " " << s.getZ() << std::endl;
    std::cout << "Suspected pose ATTITUDE: " << roll*RAD2DEG<<" " << pitch*RAD2DEG << " " << yaw*RAD2DEG << std::endl;
    std::cout << "************************************************"<< std::endl;
*/
    handler.publish("vision_position_estimate",&lcm_pose);


}


int main(int argc, char **argv)
{

    //ROS helpers
    ros::init(argc, argv, "ros2lcm_bridge");
    ros::NodeHandle n;
    ros::Subscriber odometry_sub = n.subscribe("/mavros/local_position/pose",1,&odometryCallback);
    ros::Publisher  pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);

    //LCM stuff
    lcm::LCM handler;
    lcm::Subscription *sub2 = handler.subscribe("local_position_sp", &CallbackHandler::positionSetpointCallback, &call);
    sub2->setQueueCapacity(2);

    struct pollfd fds[1];
    fds[0].fd = handler.getFileno(); // Actual task
    fds[0].events = POLLIN;

    //main loop
    ros::Rate loop_rate(30);

    while (ros::ok()){

        int ret = poll(fds,1,0);

        if(fds[0].revents & POLLIN){

            geometry_msgs::PoseStamped commandPose;
            handler.handle();

            commandPose.pose.position.x = call._position_sp.getX();
            commandPose.pose.position.y = call._position_sp.getY();
            commandPose.pose.position.z = -call._position_sp.getZ();

            commandPose.pose.orientation.x  = call._position_sp.getOrientation().x();
            commandPose.pose.orientation.y  = call._position_sp.getOrientation().y();
            commandPose.pose.orientation.z  = call._position_sp.getOrientation().z();
            commandPose.pose.orientation.w  = call._position_sp.getOrientation().w();

            pub.publish(commandPose);
            std::cout << "command: " << commandPose.pose.position.x << " " << commandPose.pose.position.y << " " <<commandPose.pose.position.z << std::endl;
            ROS_INFO_ONCE("publish ros command");

        }
        ROS_INFO_ONCE("Spinning");
        ros::spinOnce();
        loop_rate.sleep();

    }


    return 0;

}
