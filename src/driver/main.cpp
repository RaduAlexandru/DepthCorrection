#include <iostream>
#include <ros/ros.h>
#include "../shared/CalibrationMatrix.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include "driver.h"


int main(int argc, char **argv)
{
    std::cout<<"DRIVER INIT"<<std::endl;
    ros::init(argc, argv, "xtionDriver");
    ros::NodeHandle n("xtionDriver");
    std::cout<<"ROSNODE INIT"<<std::endl;

    xtionDriver d(n);

    std::cout<<"TOPIC SUBSCRIBED INIT"<<std::endl;
    ros::Subscriber s = n.subscribe("/camera/depth/image_raw", 50, &xtionDriver::callback, &d);
    ros::spin();
    return 1;
}
