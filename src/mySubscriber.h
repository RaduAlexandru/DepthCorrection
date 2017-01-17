#ifndef SUBS_H
#define SUBS_H
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
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
#include "shared/CalibrationMatrix.h"
#include "Plane.h"



class MySubscriber{
    public:
        MySubscriber(FancyViewer*);
        ~MySubscriber();
        void callback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
        void spin();
        FancyQueue* m_queue;
        float m_validNormalRange;
        Eigen::Vector4f m_planeCoefficient;
        Eigen::Vector4f m_planeCentroid;
        bool m_shutdown_required;
        ecl::Thread m_thread;
        //std::optional<CalibrationMatrix> m_multiplier;
        //CalibrationMatrix m_multiplier;
        CalibrationMatrix* m_multiplier;
        //Eigen::Matrix<double, 3, 4>  m_proj_matrix;
        bool m_first_cloud;
        float m_voxelLeaf;
        float m_normalRejection;
        bool m_show_planeModelInliers;
        bool m_computeRefenceDistance;
        int m_refenceDistance;
        int m_processing_counter;

        bool m_applyCorrection;
        bool m_recordData;


        double m_fx;
        double m_fy;
        double m_cx;
        double m_cy;

        void computePointcloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
        void computeCenterSquareCloud();
        void voxelize();
        void computeCenterCloud();
        void computerCenterPlane();
        void computeErrorPerPoint();
        void computeCalibrationMatrix();
        void calibratePointCloudWithMultipliers();
        void computeNormals();
        void pointrejection();


    private:
        //cv_bridge::CvImagePtr _image;
        FancyViewer* m_viewer;
        pcl::PointCloud<pcl::PointXYZRGB> m_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> m_correctCloud;
        pcl::PointCloud<pcl::PointXYZRGB> m_errorCloud;
        pcl::PointCloud<pcl::PointXYZRGB> m_centerCloud;
        pcl::PointCloud<pcl::Normal> m_cloud_normals;
        cv::Vec3f localToWorld(cv::Vec3f localpoint);
        std::vector<bool> m_validPoints;
        pcl::PointXYZRGB worldToImagePlane( pcl::PointXYZRGB p);
};

#endif
