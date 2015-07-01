#include <iostream>
#include <ros/ros.h>

#include "camera.h"

using namespace std;

ros::NodeHandle* ewynNode;

int main( int argc, char** argv ) {
    ros::init(argc, argv, "ewyn_ros_node");
    ewynNode = new ros::NodeHandle();
    ros::Rate loopRate(2);

    Camera camera = Camera();

    int loopCounter = 0;

    while (ros::ok()) {
        ROS_INFO("loopCounter: %d", loopCounter++);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}