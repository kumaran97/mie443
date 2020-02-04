#define N_BUMPER 3
#define RAD2DEG(rad)((rad)*180./M_PI)
#define DEG2RAD(deg)((deg)*M_PI/180.)

#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_datatypes.h>

#include <stdio.h>
#include <iostream>
#include <cmath>

#include <chrono>


float angular = 0.0;
float linear = 0.0;
float posX=0.0, posY=0.0, yaw =0.0;

float maxLaserDist = std::numeric_limits<float>::infinity();
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t maxIndex;
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;
//double laserDistLeft=0;
//double laserDistRight=0;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
	    //laserDistRight = msg->ranges[laser_idx];
            maxIndex = laser_idx;
        }
	    //laserDistLeft = msg->ranges[nLasers / 2 - desiredNLasers];
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
	    //laserDistRight = msg->ranges[laser_idx];
        }
	    //laserDistLeft = msg->ranges[nLasers / 2 - desiredNLasers];
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr&msg)
{
    posX=msg->pose.pose.position.x;
    posY=msg->pose.pose.position.y;
    yaw=tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f,%f) Orientation: %f rad or %f degrees.", posX, posY,yaw,RAD2DEG(yaw));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "maze_explorer");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("/odom",10, &odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

 //   ros::Subscriber odom=nh.subscribe("odom", 1, odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        //fill with your code
        //
        // Check if any of the bumpers were pressed.
        // bool any_bumper_pressed = false;
        // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        // any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        // }
        //
        // Control logic after bumpers are being pressed.
        // if(posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
        //     angular = 0.0;
        //     linear = 0.2;
        // }
        // else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed) {
        //     angular = M_PI / 6;
        //     linear = 0.0;
        // }
        
        int xRan;
	    srand( time(0));
	    xRan=rand()%360 - 180;


        if (minLaserDist > 0.5)
            linear = 0.1;
        else {
            linear = 0;
            angular = 0;
            float desiredYaw = yaw + M_PI / 4;
            if(desiredYaw >= M_PI) desiredYaw -= 2 * M_PI;

            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);

            int wsgn = 1;
            if (desiredYaw - yaw > 0) wsgn = -1;

            while (desiredYaw-yaw >= 0.1 || desiredYaw - yaw <= -0.1){
                ros::spinOnce();
                angular = wsgn * M_PI/12;
		    
		/*if laserDistLeft > laserDistRight {
			angular = M_PI/6;
		}
		else if laserDistRight > laserDistLeft {
			angular = -M_PI/6; 
		}
		else {
			int wsgn2 = (rand() > RAND_MAX/2) ? -1 : 1;
			angular = wsgn2 * M_PI/6;*/
			
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                ROS_INFO("ANGLE : %F  current angle %f diff %f", desiredYaw, yaw, desiredYaw - yaw);
            }
        }
            // if (yaw < 17 / 36 * M_PI || posX > 0.6) {
            //     angular = M_PI / 12.;
            // }
            // else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
            //     angular = -M_PI / 12.;
            // }
            // else {
            //     angular = 0;
        //     }
        // }
        // else {
        //     angular = 0.0;
        //     linear = 0.0;
            
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
