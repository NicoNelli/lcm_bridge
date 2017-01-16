#include "lcm/lcm-cpp.hpp"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "lcm_messages/geometry/pose.hpp"
#include "common/MavState.h"
#include "common/CallbackHandler.hpp"
#include <poll.h>
#include "nav_msgs/Odometry.h"

lcm::LCM handler, handler2;
geometry::pose lcm_pose;
CallbackHandler call;

void odometryCallback(nav_msgs::Odometry pose){

    lcm_pose.position[0] =  pose.pose.pose.position.x;
    lcm_pose.position[1] =  pose.pose.pose.position.y;
    lcm_pose.position[2] = -pose.pose.pose.position.z;

    lcm_pose.velocity[0] = pose.twist.twist.linear.x;
    lcm_pose.velocity[1] = pose.twist.twist.linear.y;
    lcm_pose.velocity[2] = pose.twist.twist.linear.z;

    lcm_pose.orientation[0] = pose.pose.pose.orientation.w;
    lcm_pose.orientation[1] = pose.pose.pose.orientation.x;
    lcm_pose.orientation[2] = pose.pose.pose.orientation.y;
    lcm_pose.orientation[3] = pose.pose.pose.orientation.z;

    handler.publish("vision_position_estimate",&lcm_pose);

}


int main(int argc, char **argv)
{

    //ROS helpers
    ros::init(argc, argv, "ros2lcm_bridge");
    ros::NodeHandle n;
    ros::Subscriber odometry_sub = n.subscribe("/mavros/local_position/odom",1,&odometryCallback);
    ros::Publisher  pub  = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
    ros::Publisher  pub2 = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);

    //LCM stuff

    lcm::Subscription *sub2 = handler2.subscribe("local_position_sp", &CallbackHandler::positionSetpointCallback, &call);
    sub2->setQueueCapacity(1);

    struct pollfd fds[1];
    fds[0].fd = handler2.getFileno(); // Actual task
    fds[0].events = POLLIN;

    //main loop
    ros::Rate loop_rate(30);

    while (ros::ok()){

        int ret = poll(fds,1,0);

        if(fds[0].revents & POLLIN){

            if(call._position_sp.getType() == MavState::type::POSITION) {
                geometry_msgs::PoseStamped commandPose;
                handler2.handle();

                commandPose.pose.position.x = call._position_sp.getX();
                commandPose.pose.position.y = call._position_sp.getY();
                commandPose.pose.position.z = -call._position_sp.getZ();

                commandPose.pose.orientation.x = call._position_sp.getOrientation().x();
                commandPose.pose.orientation.y = call._position_sp.getOrientation().y();
                commandPose.pose.orientation.z = call._position_sp.getOrientation().z();
                commandPose.pose.orientation.w = call._position_sp.getOrientation().w();


                std::cout << "command: " << commandPose.pose.position.x << " " << commandPose.pose.position.y << " " <<commandPose.pose.position.z << std::endl;
                pub.publish(commandPose);
            }
            else if(call._position_sp.getType() == MavState::type::VELOCITY) {
                geometry_msgs::TwistStamped commandPose;
                handler2.handle();

                commandPose.twist.linear.x =  call._position_sp.getVx();
                commandPose.twist.linear.y =  call._position_sp.getVy();
                commandPose.twist.linear.z =  call._position_sp.getVz();

                std::cout << "commandV: " << commandPose.twist.linear.x << " " << commandPose.twist.linear.y << " " <<commandPose.twist.linear.z << std::endl;
                pub2.publish(commandPose);
            }

            ROS_INFO_ONCE("publish ros command");

        }
        ROS_INFO_ONCE("Spinning");
        ros::spinOnce();
        loop_rate.sleep();

    }


    return 0;

}
