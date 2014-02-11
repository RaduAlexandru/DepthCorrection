#ifndef SUBS_H
#define SUBS_H
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <iostream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <ecl/threads/thread.hpp>
#include <string>
#include <eigen3/Eigen/Core>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include "fancyViewer.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <QApplication>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/geometry.h>
#include "FancyQueue.h"
#include <omp.h>
#include "CalibrationMatrix.h"
#include "Plane.h"


using namespace sensor_msgs;
using namespace message_filters;
using namespace Eigen;
using namespace std;


class MySubscriber{
    public:
        MySubscriber(FancyViewer*);
        ~MySubscriber();
        void callback(const sensor_msgs::ImageConstPtr &imgPtr);
        void spin();
        FancyQueue* queue;
        float _validNormalRange;
        Eigen::Vector4f planeCoefficient;
        Eigen::Vector4f planeCentroid;
        bool shutdown_required;
        ecl::Thread thread;
        CalibrationMatrix multiplier;
        CalibrationMatrix hits;

        void computePointcloud();
        void voxelize();
        void computeCenterCloud();
        void computerCenterPlane();
        void computeErrorPerPoint();
        void computeCalibrationMatrix();
        void calibratePointCloudWithMultipliers();
        void computeNormals();
        void pointrejection();
        bool applyCorrection;
        bool recordData;

    private:
        cv_bridge::CvImagePtr _image;
        FancyViewer* _viewer;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> correctCloud;
        pcl::PointCloud<pcl::PointXYZRGB> errorCloud;
        pcl::PointCloud<pcl::PointXYZRGB> centerCloud;
        pcl::PointCloud<pcl::Normal> cloud_normals;
        cv::Vec3f localToWorld(cv::Vec3f localpoint);
        std::vector<bool> validPoints;
        pcl::PointXYZRGB worldToImagePlane( pcl::PointXYZRGB p);
};

#endif
