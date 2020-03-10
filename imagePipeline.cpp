#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"
#include <imagePipeline.h>

using namespace cv;
using namespace cv::xfeatures2d;

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/image" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle &n)
{
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

double imgScoring(Mat img_object, Mat img_scene)
{
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create(minHessian);
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute(img_object, Mat(), keypoints_object,
                               descriptors_object);
    detector->detectAndCompute(img_scene, Mat(), keypoints_scene,
                               descriptors_scene);

    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;
    matcher.match(descriptors_object, descriptors_scene, matches);
    double max_dist = 0;
    double min_dist = 100;

    for (int i = 0; i < descriptors_object.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_object.rows; i++)
    {
        if (matches[i].distance < 3 * min_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }
    // std::vector<cv::DMatch>> matches;
    // cv::BFMatcher matcher;
    // matcher.knnMatch(descriptors_1, descriptors_2, matches, 2); // Find two nearest matches
    // vector<cv::DMatch> good_matches;
    // for (int i = 0; i < matches.size(); ++i)
    // {
    //     const float ratio = 0.8; // As in Lowe's paper; can be tuned
    //     if (matches[i][0].distance < ratio * matches[i][1].distance)
    //     {
    //         good_matches.push_back(matches[i][0]);
    //     }
    // }
    Mat img_matches;
    drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for (int i = 0; i < good_matches.size(); i++)
    {
        //-- Get the keypoints from the good matches
        obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    // check if there are 4 in obj and scene or the computer will crash
    if (obj.size() < 4 || scene.size() < 4)
    {
        std::cout << "something wrong" << std::endl;
        return 0;
    }

    Mat H = findHomography(obj, scene, RANSAC);

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0, 0);
    obj_corners[1] = cvPoint(img_object.cols, 0);
    obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
    obj_corners[3] = cvPoint(0, img_object.rows);
    std::vector<Point2f> scene_corners(4);
    try{
        perspectiveTransform(obj_corners, scene_corners, H);
    }catch(int e){
        return 0;
    }
    

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0),
         scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0),
         scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0),
         scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0),
         scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    //-- Show detected matches
    cv::imshow("Good Matches & Object detection", img_matches);
    //img_matches.convertTo
    //printf("Image vector:")
    cv::waitKey(10);

    //calculating surface

    double xa = scene_corners[0].x;
    double xb = scene_corners[1].x;
    double xc = scene_corners[2].x;
    double xd = scene_corners[3].x;

    double ya = scene_corners[0].y;
    double yb = scene_corners[1].y;
    double yc = scene_corners[2].y;
    double yd = scene_corners[3].y;

    double surface = abs((xa - xb) * (ya - yc) - (ya - yb) * (xa - xc)) +
                     abs((xa - xb) * (ya - yd) - (ya - yb) * (xa - xd)) +
                     abs((xb - xc) * (yb - yd) - (yb - yc) * (xb - xd)) +
                     abs((xc - xa) * (yc - yd) - (yc - ya) * (xc - xd));

    surface /= 2;

    double score = 0;
    score = surface;

    return score;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        if (isValid)
        {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

int ImagePipeline::getTemplateID(Boxes &boxes)
{
    int template_id = -1;
    if (!isValid)
    {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    }
    else if (img.empty() || img.rows <= 0 || img.cols <= 0)
    {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    }
    else
    {
        /***YOUR CODE HERE***/
        //write for loop to do for each template
        int j = 0;
        int weight[4] = 0;
        double raisinbranCredit = 0;
        double cinnamoncrunchCredit = 0;
        double ricecrispiesCredit = 0;
        double blank = 0;
        while (j<6){
            for (int i = 0; i < 3; i++)
            {
                Mat img_object = boxes.templates[i];
                Mat img_scene = img;
                double score = imgScoring(img_object, img_scene);
                printf("score %lf", score);
                if(i==0 && score >= 3000){
                    weight[0] += score;
                    //raisinbranCredit++;
                    printf("RB %d \n", i);}
                elseif(i==1 && score>=3000){
                    weight[1] += score;
                    //cinnamoncrunchCredit++;
                    printf("CC %d \n", i);}
                elseif(i==2 && score>=3000){
                    weight[2] += score;
                    //ricecrispiesCredit++;
                    printf("RC %d \n", i);}
                else{
                    weight[3] += score;
                    blank++;
                    printf("empty");}
            }
        cv::imshow("view", img);
        cv::waitKey(10);
        j++;            
        }
        int max_index = 0;
        int max = 0;
        for (i=0, i<4, i++){ 
            if (weight[i] > max){
                max = weight[i];
                max_index = i;}
        }
        if (max_index == 0)
            printf("raisin bran %d \n", i);
        elseif (max_index == 1)
            printf("cinnamon crunch %d \n", i);
        elseif (max_index == 2)
            printf("rice crispies %d \n", i);
        else
            printf("Blank Scene, put the right cereal box.");
        
    }
    return template_id;
}
