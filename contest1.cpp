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
int32_t nLasers=0, desiredNLasers=0, desiredAngle=15;

double laserDistLeftGlobal = 0.0;
double laserDistRightGlobal = 0.0;



void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
}



void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    minLaserDist = msg->ranges[nLasers / 2 - desiredNLasers];
    maxLaserDist = msg->ranges[nLasers / 2 - desiredNLasers];

    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
            maxIndex = laser_idx;
            // laserDistRightGlobal = (msg->ranges[laser_idx]+msg->ranges[laser_idx-1]+msg->ranges[laser_idx-2])/3;
            laserDistRightGlobal = msg->ranges[laser_idx];
            
        }
        
        //laserDistLeftGlobal = (msg->ranges[nLasers / 2 - desiredNLasers]+msg->ranges[nLasers / 2 - desiredNLasers+1]+msg->ranges[nLasers / 2 - desiredNLasers+2])/3;
        laserDistLeftGlobal = msg->ranges[nLasers / 2 - desiredNLasers];
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
            // laserDistRightGlobal = (msg->ranges[laser_idx]+msg->ranges[laser_idx-1]+msg->ranges[laser_idx-2])/3;
            laserDistRightGlobal = msg->ranges[laser_idx];
            
        
        }
        // laserDistLeftGlobal = (msg->ranges[1]+msg->ranges[2]+msg->ranges[3])/3;
        laserDistLeftGlobal = msg->ranges[1];
        
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr&msg)
{
    posX=msg->pose.pose.position.x;
    posY=msg->pose.pose.position.y;
    yaw=tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
    // ROS_INFO("Position: (%f,%f) Orientation: %f rad or %f degrees.", posX, posY,yaw,RAD2DEG(yaw));
}

double turnDirection(double laserDistLeft, double laserDistRight)
{
    double angular1 = 0.0;
    if (laserDistLeft - laserDistRight > 0.0001) {
        angular1 = -M_PI/6;
    }
    else if (laserDistRight - laserDistLeft > 0.0001) {
        angular1 = M_PI/6;
    }
    else {
        angular1 = -M_PI/6;
    }
    return angular1;

}

double desiredYawGlobal = 0.0;

double orientation(double maxLaserDist, double laserDistLeftGlobal, double laserDistRightGlobal, double yaw)
{ double desiredYaw = 0.0;
    if (maxLaserDist < 1 && laserDistLeftGlobal - laserDistRightGlobal > 0.0001) {
        desiredYaw = yaw - M_PI / 6;
    }
    else if (maxLaserDist < 1 && laserDistRightGlobal - laserDistLeftGlobal > 0.0001) {
        desiredYaw = yaw + M_PI / 6;   
    }
    else {
        desiredYaw = yaw - 3*M_PI/4;
    }
    return desiredYaw;
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
                
        // int xRan;
	    // srand( time(0));
	    // xRan=rand()%360 - 180;

        ROS_INFO("minLaserDist = %f", minLaserDist);
        if (minLaserDist > 0.5){
            linear = 0.25;
            angular = 0;
        }
        else {
            linear = 0;
            angular = 0;
            // float desiredYaw = yaw + M_PI / 4;
            // if(desiredYaw >= M_PI) desiredYaw -= 2 * M_PI;
            desiredYawGlobal = orientation(maxLaserDist, laserDistLeftGlobal, laserDistRightGlobal, yaw);

            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);

            // int wsgn = 1;
            // if (desiredYaw - yaw < 0 && desiredYaw - yaw > - M_PI) wsgn = -1;

            while (desiredYawGlobal-yaw >= 0.1 || desiredYawGlobal - yaw <= -0.1){
                ros::spinOnce();
                // angular = wsgn * M_PI/6;
                angular = turnDirection(laserDistLeftGlobal, laserDistRightGlobal);
                ROS_INFO("Left : %f  Right %f", laserDistLeftGlobal, laserDistRightGlobal);
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                ROS_INFO("ANGLE : %f  current angle %f diff %f", desiredYawGlobal, yaw, desiredYawGlobal - yaw);
                loop_rate.sleep();
            }

            //ROS_INFO("Out! %f", desiredYawGlobal - yaw);
        }
            
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
