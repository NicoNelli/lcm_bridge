#include "lcm/lcm-cpp.hpp"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/ParamSet.h"
#include "lcm_messages/geometry/pose.hpp"
#include "lcm_messages/geometry/vision.hpp"
#include "lcm_messages/exec/state.hpp"
#include "common/MavState.h"
#include "common/CallbackHandler.hpp"
#include "nav_msgs/Odometry.h"
#include <poll.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "apriltags/AprilTagDetections.h"
#include <cmath>
#include <queue>
#include <string>

lcm::LCM handler, handler2, handler3, handler4;
geometry::pose lcm_pose;
exec::state robot_state;
nav_msgs::Odometry platPos;
mavros_msgs::State disarmed;
mavros_msgs::ExtendedState landed;
CallbackHandler call;

geometry::vision vision_pose; //lcm message for vision topic.

//To estimate roll&pitch&yaw.
typedef std::queue<double> queue_t;
queue_t Queue_roll;
queue_t Queue_pitch;
queue_t Queue_yaw;
double integral_roll = 0;
double integral_pitch = 0;
double integral_yaw = 0;
double roll;
double pitch;
double yaw;

/*
bool drone = false;
bool platform = false;
Eigen::Matrix4d zero_T_drone;
Eigen::Matrix4d zero_T_plat;
Eigen::Matrix4d drone_T_plat;
*/

bool firstState = true;
bool firstEState = true;

std::string mode;

void odometryCallback(nav_msgs::Odometry pose){

    lcm_pose.position[0] =  pose.pose.pose.position.x;
    lcm_pose.position[1] =  pose.pose.pose.position.y;
    lcm_pose.position[2] =  pose.pose.pose.position.z;

    lcm_pose.velocity[0] = pose.twist.twist.linear.x;
    lcm_pose.velocity[1] = pose.twist.twist.linear.y;
    lcm_pose.velocity[2] = pose.twist.twist.linear.z;

    lcm_pose.orientation[0] = pose.pose.pose.orientation.w;
    lcm_pose.orientation[1] = pose.pose.pose.orientation.x;
    lcm_pose.orientation[2] = pose.pose.pose.orientation.y;
    lcm_pose.orientation[3] = pose.pose.pose.orientation.z;


    if( !mode.compare("OFFBOARD") )
		lcm_pose.isValid = 1;
	else
		lcm_pose.isValid = 0;

	std::cout<<"valid: "<<(unsigned)lcm_pose.isValid<<std::endl;

    //Here i have to build the transformation matrix between the drone and absolute reference frame
    /*
    Eigen::Quaterniond q2;
    q2.x() = pose.pose.pose.orientation.x;
    q2.y() = pose.pose.pose.orientation.y;
    q2.z() = pose.pose.pose.orientation.z;
    q2.w() = pose.pose.pose.orientation.w;

    Eigen:: Matrix3d R2 = q2.normalized().toRotationMatrix();

    zero_T_drone << R2(0,0), R2(0,1), R2(0,2), lcm_pose.position[0],
                    R2(1,0), R2(1,1), R2(1,2), lcm_pose.position[1], 
                    R2(2,0), R2(2,1), R2(2,2), lcm_pose.position[2],
                          0,       0,       0,                    1;

    zero_T_drone = zero_T_drone.inverse().eval();
    drone = true;
*/

    handler.publish("vision_position_estimate",&lcm_pose);

}

double movingFilter(double raw, std::queue<double> *queue_t, double *integral){

    int window = 12;
    if( queue_t->size() == window){

        *integral += raw - queue_t->front(); //return the oldest element of the FIFO queue
        queue_t->pop();

    }
    else
        *integral += raw;

    queue_t->push(raw);
    
    return *integral/queue_t->size();


}

void toEulerAngle(double quaternion[], double *Roll, double *Pitch, double *Yaw){

    //roll (x-axis rotation)
    double sinr_cosp = 2.0*( quaternion[3]*quaternion[0] + quaternion[1]*quaternion[2] );
    double cosr_cosp = 1.0 - 2.0*( quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] );

    *Roll = atan(sinr_cosp/cosr_cosp); 

    //pitch (y-axis rotation)
    double sinp = 2.0*( quaternion[3]*quaternion[1] - quaternion[2]*quaternion[0] );
    
    if(fabs(sinp) >= 1)
        *Pitch = copysign(M_PI/2,sinp); //use 90 degrees if out of range
    else
        *Pitch = asin(sinp);

    //yaw (z-axis rotation)
    double siny_cosp = 2.0*( quaternion[3]*quaternion[2] + quaternion[0]*quaternion[1] );
    double cosy_cosp = 1.0 -2.0*( quaternion[1]*quaternion[1] + quaternion[3]*quaternion[3] );
    *Yaw = atan2(siny_cosp,cosy_cosp);

}



void stateCallback(mavros_msgs::State s){

    if (firstState) {
        disarmed.armed = s.armed;
        firstState = false;
    }

    mode = s.mode;

    //Store arming state
    if(disarmed.armed == s.armed) robot_state.armed = 0;
    else robot_state.armed = 1;

}
void EStateCallback(mavros_msgs::ExtendedState es){

    if (firstEState) {
        landed.landed_state = es.landed_state;
        firstEState = false;
    }

    if(landed.landed_state == es.landed_state) robot_state.landed = 1;
    else robot_state.landed = 0;

}

void ApriltagCallback(apriltags::AprilTagDetections A_det){
			
        if( A_det.detections.size() >0 ){ //check if there is at least one detected tag.

                Eigen::MatrixXd AreaID(A_det.detections.size(),2); //in each row, there is the area of the tag and its index in the vector.

                Eigen::Vector3d rel_pose; //relative position choosen.

                for(int i=0; i< A_det.detections.size(); i++){ //for each tag, it computes each side of the square.
		
                        double l1 = sqrt(pow(A_det.detections[i].corners2d[1].y - A_det.detections[i].corners2d[0].y, 2)+pow(A_det.detections[i].corners2d[1].x - A_det.detections[i].corners2d[0].x,2));
		
                        double l2 = sqrt(pow(A_det.detections[i].corners2d[2].y - A_det.detections[i].corners2d[1].y, 2)+pow(A_det.detections[i].corners2d[2].x - A_det.detections[i].corners2d[1].x,2));
	
                        double l3 = sqrt(pow(A_det.detections[i].corners2d[3].y - A_det.detections[i].corners2d[2].y, 2)+pow(A_det.detections[i].corners2d[3].x - A_det.detections[i].corners2d[2].x,2));

                        double l4 = sqrt(pow(A_det.detections[i].corners2d[0].y - A_det.detections[i].corners2d[3].y, 2)+pow(A_det.detections[i].corners2d[0].x - A_det.detections[i].corners2d[3].x,2));
		
                        double meanL = (l1 + l2 + l3 + l4)/4; //mean value of the lenght of the side.

                        //filled the matrix 
                        AreaID(i,0) = i;
                        AreaID(i,1) = pow(meanL,2); //area of the tag.
                }

                //now an avarage weighted to estimate the position.

                double WeightTot = 0;
				for(int i =0; i <AreaID.rows(); i++){
                	vision_pose.position[0] += A_det.detections[AreaID(i,0)].pose.position.x * AreaID(i,1);	
                	vision_pose.position[1] += A_det.detections[AreaID(i,0)].pose.position.y * AreaID(i,1);
                	vision_pose.position[2] += A_det.detections[AreaID(i,0)].pose.position.z * AreaID(i,1);
                	
                	vision_pose.orientation[0] += A_det.detections[AreaID(i,0)].pose.orientation.x * AreaID(i,1);
                 	vision_pose.orientation[1] += A_det.detections[AreaID(i,0)].pose.orientation.y * AreaID(i,1);
                 	vision_pose.orientation[2] += A_det.detections[AreaID(i,0)].pose.orientation.z * AreaID(i,1);
                	vision_pose.orientation[3] += A_det.detections[AreaID(i,0)].pose.orientation.w * AreaID(i,1);

                	WeightTot +=AreaID(i,1);
                }
				

                vision_pose.position[0] = (vision_pose.position[0])/WeightTot;
                vision_pose.position[1] = (vision_pose.position[1])/WeightTot;
                vision_pose.position[2] = (vision_pose.position[2])/WeightTot;

                vision_pose.velocity[0] = 0;
                vision_pose.velocity[1] = 0;
                vision_pose.velocity[2] = 0;

                vision_pose.orientation[0] = (vision_pose.orientation[0])/WeightTot; 
                vision_pose.orientation[1] = (vision_pose.orientation[1])/WeightTot; 
                vision_pose.orientation[2] = (vision_pose.orientation[2])/WeightTot;
                vision_pose.orientation[3] = (vision_pose.orientation[3])/WeightTot;

                toEulerAngle(vision_pose.orientation, &roll, &pitch, &yaw);
                //moving filter to clean a bit the data

            	//DEBUG
            	//std::cout<<"x: "<<vision_pose.position[0]<<std::endl; 
                //std::cout<<"y: "<<vision_pose.position[1]<<std::endl;           
				//std::cout<<"z: "<<vision_pose.position[2]<<std::endl;
                vision_pose.roll = movingFilter(roll, &Queue_roll, &integral_roll);
                vision_pose.pitch = movingFilter(pitch, &Queue_pitch, &integral_pitch);
                vision_pose.yaw = movingFilter(yaw, &Queue_yaw, &integral_yaw);


               handler4.publish("apriltag_vision_system",&vision_pose);
               //lcm publication.

	}
	
}

int main(int argc, char **argv)
{

    //ROS helpers
    ros::init(argc, argv, "ros2lcm_bridge");
    ros::NodeHandle n;

    ros::Subscriber odometry_sub = n.subscribe("/mavros/local_position/odom",1,&odometryCallback);//ros topic for odom of the robot.
    ros::Subscriber state_sub = n.subscribe("/mavros/state",1,&stateCallback);
    ros::Subscriber state_extended_sub = n.subscribe("/mavros/extended_state",1,&EStateCallback);
    ros::Subscriber relative_pose_sub = n.subscribe("/apriltags/detections",1,&ApriltagCallback);//apriltags system.


    ros::Publisher  pub  = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
    ros::Publisher  pub1 = n.advertise<nav_msgs::Odometry>("/global_platform_position",1);
    ros::Publisher  pub2 = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);

    //ros::Subscriber PLatformPose_sub = n.subscribe("/global_platform_position",1,&PlatformCallback);


    //LCM stuff
    lcm::Subscription *sub2 = handler2.subscribe("local_position_sp", &CallbackHandler::positionSetpointCallback, &call);
    lcm::Subscription *sub3 = handler3.subscribe("Landing_Site/pose"    , &CallbackHandler::visionEstimateCallback, &call);
    sub2->setQueueCapacity(1);
    sub3->setQueueCapacity(1);

    struct pollfd fds[2];
    fds[0].fd = handler2.getFileno(); // Robot position
    fds[0].events = POLLIN;

    fds[1].fd = handler3.getFileno(); // Platform position
    fds[1].events = POLLIN;

    robot_state.landed = 1;
    robot_state.armed  = 0;

    //main loop
    ros::Rate loop_rate(30);
    int stateRate = 0;
    int*  platformDataRec = new int(0);
    int*  robotDataRec    = new int(0);

    while (ros::ok()){

        //Poll file descriptors
        int ret = poll(fds,2,0);
        *platformDataRec = fds[1].revents & POLLIN;
        *robotDataRec    = fds[0].revents & POLLIN;

        //Platform position POLLIN
        if(*platformDataRec){

            handler3.handle();
            platPos.pose.pose.position.x = call._vision_pos.getX();
            platPos.pose.pose.position.y = call._vision_pos.getY();
            platPos.pose.pose.position.z = call._vision_pos.getZ();

            platPos.twist.twist.linear.x = call._vision_pos.getVx();
            platPos.twist.twist.linear.y = call._vision_pos.getVy();
            platPos.twist.twist.linear.z = call._vision_pos.getVz();

            platPos.header.stamp = ros::Time::now();
            pub1.publish(platPos);

        }

        //Position Command POLLIN
        if(*robotDataRec){

            if(call._position_sp.getType() == MavState::type::POSITION) {
                geometry_msgs::PoseStamped commandPose;
                handler2.handle();

                commandPose.pose.position.x = call._position_sp.getX();
                commandPose.pose.position.y = call._position_sp.getY();
                commandPose.pose.position.z = call._position_sp.getZ();

                commandPose.pose.orientation.x = call._position_sp.getOrientation().x();
                commandPose.pose.orientation.y = call._position_sp.getOrientation().y();
                commandPose.pose.orientation.z = call._position_sp.getOrientation().z();
                commandPose.pose.orientation.w = call._position_sp.getOrientation().w();

                //std::cout << "command: " << commandPose.pose.position.x << " " << commandPose.pose.position.y << " " <<commandPose.pose.position.z << std::endl;
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

        //Publish state sometimes
        if (stateRate++ > 10){
            handler2.publish("state",&robot_state);
            stateRate = 0;
        }


        ROS_INFO_ONCE("Spinning");
        ros::spinOnce();
        loop_rate.sleep();

    }

    delete platformDataRec;
    delete robotDataRec;

    return 0;

}


/*
void PlatformCallback(geometry_msgs::PoseStamped msg){

    //Here i have to build the transformation matrix between the drone and absolute reference frame
    
    Eigen::Quaterniond q3;
    q3.x() = msg.pose.orientation.x;
    q3.y() = msg.pose.orientation.y;
    q3.z() = msg.pose.orientation.z;
    q3.w() = msg.pose.orientation.w;

    Eigen:: Matrix3d R3 = q3.normalized().toRotationMatrix();

    zero_T_plat << R3(0,0), R3(0,1), R3(0,2), msg.pose.position.x,
                   R3(1,0), R3(1,1), R3(1,2), msg.pose.position.y, 
                   R3(2,0), R3(2,1), R3(2,2), msg.pose.position.z,
                          0,       0,       0,                    1;

    bool platform = true;

    if(drone && platform){

        drone_T_plat = zero_T_drone * zero_T_plat;

        Eigen::RowVectorXd error(3);
        error << drone_T_plat(0,3), drone_T_plat(1,3),drone_T_plat(2,3);
        std::cout << "error: " << error << std::endl; 

    }

}
*/