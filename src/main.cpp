#include <iostream>
#include <ros/ros.h>
#include "CalibrationMatrix.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include "FancyQueue.h"
#include <string>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <QApplication>
#include "fancyWindow.h"
#include "CalibrationMatrix.h"
#include "mySubscriber.h"


int main(int argc, char **argv)
{
    std::cout<<"INIT"<<std::endl;
    ros::init(argc, argv, "xtionCalib");
    ros::NodeHandle n("xtionCalib");
    std::cout<<"ROSNODE INIT"<<std::endl;

    QApplication qapp(argc, argv);
    FancyWindow w;
    FancyQueue queue;
    MySubscriber mySub(w.viewer);
    mySub.queue=&queue;
    w.viewer->queue=&queue;
    w.sub=&mySub;



    std::cout<<"TOPIC SUBSCRIBED INIT"<<std::endl;
    ros::Subscriber s = n.subscribe("/camera/depth/image_raw", 5, &MySubscriber::callback, &mySub);
    w.show();
    return qapp.exec();
}
