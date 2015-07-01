#include "camera.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stdint.h>
#include <string>
#include <unistd.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

Camera::Camera() {
    processId_ = pthread_create(&processThread_, NULL, process_, this);
    if (processId_) {
        throw string("Unable to create Camera thread");
    }

    ROS_INFO("Camera instantiated");
}

extern ros::NodeHandle* ewynNode;

void *Camera::process_(void* cameraPtr) {
    static unsigned int seq = 0;
    static ros::Publisher originalImagePublisher = ewynNode->advertise<sensor_msgs::Image>("EwynOriginal", 2);
    static ros::Publisher processedImagePublisher = ewynNode->advertise<sensor_msgs::Image>("EwynProcessed", 2);

    VideoCapture cap(0); //capture the video from web cam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         throw string("Cannot open the web cam");
    }

    ROS_INFO("Camera begin video capture");

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0; 
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    while (true) {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        if (!bSuccess) {
            //if not success, break loop 
            ROS_INFO("Cannot read a frame from video stream");
            break;
        }

        cv_bridge::CvImage rosOriginalImage;
        rosOriginalImage.header.seq = seq++;
        rosOriginalImage.header.stamp = ros::Time::now();
        rosOriginalImage.header.frame_id = "Ewyn original";
        rosOriginalImage.encoding = sensor_msgs::image_encodings::BGR8;
        rosOriginalImage.image = imgOriginal;
        originalImagePublisher.publish(rosOriginalImage);

        Mat imgHSV;

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
          
        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        Mat result;
        cvtColor(imgThresholded, result, COLOR_GRAY2BGR);
        cv_bridge::CvImage rosProcessedImage;
        rosProcessedImage.header.seq = rosOriginalImage.header.seq;
        rosProcessedImage.header.stamp = rosOriginalImage.header.stamp;
        rosProcessedImage.header.frame_id = "Ewyn processed";
        rosProcessedImage.encoding = sensor_msgs::image_encodings::BGR8;
        rosProcessedImage.image = result;
        processedImagePublisher.publish(rosProcessedImage);

        imshow("Thresholded Image", imgThresholded); //show the thresholded image
        imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break; 
        }
    }
}