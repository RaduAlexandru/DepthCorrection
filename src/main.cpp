#include <iostream>
#include <ros/ros.h>
#include "shared/CalibrationMatrix.h"
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
#include "shared/CalibrationMatrix.h"
#include "mySubscriber.h"

class MyApplication : public QApplication
{
public:
    MyApplication(int &argc, char **argv):QApplication ( argc, argv ) {};
    // ~MyApplication();
private:
    bool notify(QObject *receiver_, QEvent *event_)
    {
      try
      {
        return QApplication::notify(receiver_, event_);
      }
      catch (std::exception &ex)
      {
        std::cerr << "std::exception was caught at"<<&ex << std::endl;
      }

      return false;
    }
};

int main(int argc, char **argv)
{
    std::cout<<"INIT"<<std::endl;
    ros::init(argc, argv, "xtionCalib");
    ros::NodeHandle n("xtionCalib");
    std::cout<<"ROSNODE INIT"<<std::endl;

    MyApplication qapp(argc, argv);
    FancyWindow w;
    FancyQueue queue;
    MySubscriber mySub(w.viewer);
    mySub.m_queue=&queue;
    w.viewer->queue=&queue;
    w.sub=&mySub;


    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n, "/kinect2/qhd/camera_info", 3);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/kinect2/qhd/points", 3);

   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> AsyncPolicy;
   message_filters::Synchronizer<AsyncPolicy> sync(AsyncPolicy(10), info_sub, cloud_sub);
   sync.registerCallback(boost::bind(&MySubscriber::callback ,&mySub, _1, _2));



    //std::cout<<"TOPIC SUBSCRIBED INIT "<<std::endl;
    //std::string topic_name= "/kinect2/qhd/image_depth_rect";

    //ros::Subscriber s = n.subscribe(topic_name, 5, &MySubscriber::callback, &mySub);
    w.show();
    return qapp.exec();
}
