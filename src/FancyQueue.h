#ifndef FANCYQ_H
#define FANCYQ_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <queue>


class QueuePayload{
public:
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointCloud<pcl::PointXYZRGB> correctCloud;
    pcl::PointCloud<pcl::PointXYZRGB> errorCloud;
    pcl::PointCloud<pcl::Normal> normals;
    Eigen::Vector4f planeCoefficient;
    Eigen::Vector4f planeCentroid;
    std::vector<bool> validPoints;
};

class FancyQueue{
public:
    FancyQueue();
    QueuePayload data;
    void lock();
    void unlock();
    bool status();
    bool locked;
};

#endif
