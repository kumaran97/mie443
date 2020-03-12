#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

#include <cmath>

double xGoal = 0;
double yGoal = 0;
double phiGoal = 0;
double PI = 1.570795;

int cost = 0;
int completed[5] = 0;

int least(int c) {
    int nc = 999;
    int min = 999;
    int kmin = 0;
    
    for (int i = 0; i < 5; i++) {
        if ((distanceArray[c][i]!=0)&&(completed[i]==0))
            if (distanceArray[c][i] + distanceArray[i][c] < min) {
                min = distanceArray[i][0]+distanceArray[c][i];
                kmin = distanceArray[c][i];
                nc = i;
            }
    }
    
    if(min!=999) {
        cost+=kmin;
    }
    return nc;
}

int minCost(int city) {

    int ncity = 0;
    int dest = 0;
    completed[city] = 1;
    
    ncity = least(city);
    if (ncity = 999) {
        ncity = 0;
        dest = ncity + 1;
        return dest;
        cost += distanceArray[city][ncity];
    }
    mincost(ncity);
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    //Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    while (robotPose.x==0 && robotPose.y==0 && robotPose.phi==0){
        ros::spinOnce();
    }
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi = robotPose.phi;
    // // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    //ImagePipeline imagePipeline(n);
    // Execute strategy.
    
    while(ros::ok()) {
        ros::spinOnce();
        // /***YOUR CODE HERE***/
        
        //Set locations to travel to in order to take image
        float destination[5][3] = {0};
        destination[0][0] = startX;
        destination[0][1] = startY;
        destination[0][2] = startPhi;
        
        for(int i = 0; i < 5;i++) {
            phiGoal = boxes.coords[i][2] - 2*PI;
            xGoal = boxes.coords[i][0] - 0.7*cos(phiGoal);
            yGoal = boxes.coords[i][1] - 0.7*sin(phiGoal);
            destination[i+1][0] = xGoal;
            destination[i+1][1] = yGoal;
            destination[i+1][2] = phiGoal;
            completed[i] = 0;
            //Navigation::moveToGoal(xGoal,yGoal,phiGoal);
        // //std::cout << i << " Goal: " << i << std:endl;

        // Create array to store distance between all vertices
        double distanceArray[6][6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                distanceArray[i][j] = sqrt((destination[j][0] - destination[i][0])*(destination[j][0] - destination[i][0]) + 
                (destination[j][1] - destination[i][1])*(destination[j][1] - destination[i][1]));
                std::cout << distanceArray[i][j];
            }
        }
        
        }

        Navigation::moveToGoal(startX,startY,startPhi);
        //imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
        //break;
    }
    return 0;
}
