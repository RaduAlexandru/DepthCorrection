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
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

cv_bridge::CvImagePtr cvImageFromROS;
cv_bridge::CvImage out_msg;
image_transport::ImageTransport* it;
image_transport::ImageTransport* test;
image_transport::Publisher pub;
cv::Mat image;

void callback(const sensor_msgs::ImageConstPtr &imgPtr){
    cvImageFromROS =cv_bridge::toCvCopy(imgPtr);

    cvImageFromROS->image.copyTo(image);
    out_msg.header   = imgPtr->header;
    out_msg.encoding = "mono16";
    out_msg.image    = image;
    pub.publish(out_msg.toImageMsg());
}



int main(int argc, char **argv)
{
    std::cout<<"DRIVER INIT"<<std::endl;
    ros::init(argc, argv, "xtionDriver");
    ros::NodeHandle n("xtionDriver");
    std::cout<<"ROSNODE INIT"<<std::endl;

    //xtionDriver d(n);

    it= new image_transport::ImageTransport(n);
    test= new image_transport::ImageTransport(n);




    pub = it->advertise("/camera/malcom", 10);

    std::cout<<"TOPIC SUBSCRIBED INIT"<<std::endl;
    //ros::Subscriber s = n.subscribe("/camera/depth/image_raw", 50, &xtionDriver::callback, &d);
    image_transport::Subscriber sub = test->subscribe("/camera/depth/image_raw", 1, &callback);
    //ros::Subscriber s = n.subscribe("/camera/depth/image_raw", 50, &callback);
    ros::spin();
    return 1;
}


