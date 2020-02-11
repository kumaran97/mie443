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
int32_t nLasers=0, desiredNLasers=0, desiredAngle=22;

double laserDistLeftGlobal = 0.0;
double laserDistRightGlobal = 0.0;
double laserDistCenterGlobal = 0.0;

int corner = 0;         //corner = 1 if it is a corner

bool bumperLeft = 0;
bool bumperRight = 0;
bool bumperCentre = 0;
bool bumperHit = false;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr msg){
    if(msg->bumper == 0)
        bumperLeft = !bumperLeft;
    else if(msg->bumper == 1)
        bumperCentre = !bumperCentre;
    else if(msg->bumper == 2)
        bumperRight = !bumperRight;
}




void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    
    minLaserDist = msg->ranges[nLasers / 2 - desiredNLasers];
    maxLaserDist = msg->ranges[nLasers / 2 - desiredNLasers];
    
    laserDistLeftGlobal = -1;
    laserDistRightGlobal = -1;

    corner = 1;

    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            if(std::isnan(msg->ranges[laser_idx])) {
            
            continue;
            }
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);

            if (!std::isnan(msg->ranges[laser_idx])){
                laserDistRightGlobal = msg->ranges[laser_idx];
                if(laserDistLeftGlobal < 0)
                    laserDistLeftGlobal = msg->ranges[laser_idx];
            }
            if(msg->ranges[laser_idx] > 1) corner = 0;
        }
        
        laserDistCenterGlobal = msg->ranges[nLasers /2];
        
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            if(std::isnan(msg->ranges[laser_idx])) continue;
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
            
            if (!std::isnan(msg->ranges[laser_idx])){
                laserDistRightGlobal = msg->ranges[laser_idx];
                if(laserDistLeftGlobal < 0)
                    laserDistLeftGlobal = msg->ranges[laser_idx];
            }
            if(msg->ranges[laser_idx] > 1) corner = 0;
        
        }
        laserDistCenterGlobal = msg->ranges[nLasers /2];
        
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr&msg)
{
    posX=msg->pose.pose.position.x;
    posY=msg->pose.pose.position.y;
    yaw=tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
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

double orientation(double laserDistLeftGlobal, double laserDistRightGlobal, double yaw, int corner)
{ double desiredYaw = 0.0;
    if(corner == 1){
        desiredYaw = yaw - M_PI/2;
    }else if (laserDistLeftGlobal - laserDistRightGlobal > 0.0001) {
        desiredYaw = yaw - M_PI / 12;
    }
    else if (laserDistRightGlobal - laserDistLeftGlobal > 0.0001) {
        desiredYaw = yaw + M_PI / 12;   
    }
    
    if(desiredYaw > M_PI) desiredYaw -= 2 * M_PI;
    else if(desiredYaw < - M_PI) desiredYaw += 2 * M_PI;
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
        if ((secondsElapsed%60 == 0 || secondsElapsed == 0) && corner == 0){
            
            desiredYawGlobal = yaw + 3*M_PI / 2;
            if(desiredYawGlobal > M_PI) desiredYawGlobal -= 2 * M_PI;
            else if(desiredYawGlobal < - M_PI) desiredYawGlobal += 2 * M_PI;
            linear = 0;

             while (desiredYawGlobal-yaw >= 0.1 || desiredYawGlobal - yaw <= -0.1 ){
                ros::spinOnce();
                
                angular = M_PI/6;
                
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                
                loop_rate.sleep();
            }
            angular = 0;

        }

        ROS_INFO("minLaserDist = %f, centerDist = %f", minLaserDist, laserDistCenterGlobal);

        //bumper code
        if(bumperCentre == 1 || bumperLeft == 1 || bumperRight == 1){
            linear = 0;
            bumperHit = true;
            ROS_INFO("Centre Bumper Hit");
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel); 
        
            uint64_t secondsElapsedBumper = 0;
                std::chrono::time_point<std::chrono::system_clock> start_bumper;
            start_bumper = std::chrono::system_clock::now();
            
            while (secondsElapsedBumper <= 4 && bumperHit == true){
                ros::spinOnce();
                linear = -0.1;
                 if (secondsElapsedBumper > 2) {
                    linear = 0;
                    angular = turnDirection(laserDistLeftGlobal, laserDistRightGlobal);
                    
                 }

                     vel.angular.z = angular;
                     vel.linear.x = linear;
                     vel_pub.publish(vel);  
                secondsElapsedBumper = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start_bumper).count(); 
               
            }
            angular = 0;
            bumperHit = false;
            
            loop_rate.sleep();
        }

        if (minLaserDist > 0.65){
            linear = 0.2;
            angular = 0;
        }

        else if(!std::isnan(minLaserDist)){
            linear = 0;
            angular = 0;
            
            desiredYawGlobal = orientation(laserDistLeftGlobal, laserDistRightGlobal, yaw, corner);
            ROS_INFO("Cornered: %d", corner);
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);

            double initial_angular = turnDirection(laserDistLeftGlobal, laserDistRightGlobal);

            while (desiredYawGlobal-yaw >= 0.1 || desiredYawGlobal - yaw <= -0.1 ){
                ros::spinOnce();
                
                angular = initial_angular;
                
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                
                loop_rate.sleep();
            }
            angular = 0;
            
        }

        else if (std::isnan(minLaserDist)) {
            while (std::isnan(minLaserDist)) {
                ros::spinOnce();
                linear = 0;
                angular = M_PI / 10;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
            }
        }

        else ros::spinOnce();
           
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
