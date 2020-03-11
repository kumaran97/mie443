#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

#include <cmath>

double xGoal = 0;
double yGoal = 0;
double phiGoal = 0;
double PI = 1.570795;

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    // RobotPose robotPose(0,0,0);
    // ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    // while (robotPose.x==0 && robotPose.y==0 && robotPose.phi==0){
    //     ros::spinOnce();
    // }
    // float startX = robotPose.x;
    // float startY = robotPose.y;
    // float startPhi = robotPose.phi;
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
    ImagePipeline imagePipeline(n);
    // Execute strategy.
    
    //Navigation::moveToGoal(-1.404,2.5,-1.606);
    while(ros::ok()) {
        ros::spinOnce();
        // /***YOUR CODE HERE***
       
        //Set locations to travel to in order to take image of boxes
        float destination[] = {0,0,0};
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
        }
        
        // Create array to store distance between all vertices
        float distanceArray[6][6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                distanceArray[i][j] = sqrt((destination[j][0] - destination[i][0])^2 + (destination[j][1] - destination[i][1])^2);
            }
        }

        // Navigation::moveToGoal(startX,startY,startPhi);
        imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
        //break;
    }
    return 0;
}
