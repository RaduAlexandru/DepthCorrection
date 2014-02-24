#include <getopt.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "../shared/CalibrationMatrix.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <stdio.h>
#include <sensor_msgs/image_encodings.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <omp.h>

pcl::PointCloud<pcl::PointXYZRGB> cloud;
pcl::PointCloud<pcl::PointXYZRGB> correctCloud;
pcl::PointCloud<pcl::PointXYZRGB> errorCloud;
pcl::PointCloud<pcl::PointXYZRGB> centerCloud;
pcl::PointCloud<pcl::Normal> cloud_normals;
Eigen::Vector4f planeCoefficient;
Eigen::Vector4f planeCentroid;
std::vector<bool> validPoints;
CalibrationMatrix* multiplier;


void computeCalibrationMatrix();
void computeErrorPerPoint();
void computePointcloud();
void processImage(cv::Mat& image);
void computeCenterSquareCloud();
void computeNormals();
void computerCenterPlane();
int pgmfilter(const struct dirent *dir);
cv::Vec3f localToWorld(cv::Vec3f localpoint);
void voxelize(float);
void pointrejection(float);
pcl::PointXYZRGB  worldToImagePlane( pcl::PointXYZRGB p);

cv::Mat _image;

void processImage(cv::Mat& image, float v,float n){
    _image=image;
    computePointcloud();
    voxelize(v);
    computeCenterSquareCloud();
    computerCenterPlane();
    computeNormals();
    pointrejection(n);
    computeErrorPerPoint();
    computeCalibrationMatrix();

}

cv::Vec3f localToWorld(cv::Vec3f localpoint){
    float fx_d=577.30;
    float fy_d=579.41;
    float cx_d=319.5;
    float cy_d=239.5;
    cv::Vec3f worldpoint;
    worldpoint[0]=(localpoint[0]-cx_d)*localpoint[2]/fx_d;
    worldpoint[1]=(localpoint[1]-cy_d)*localpoint[2]/fy_d;
    worldpoint[2]=localpoint[2];
    return worldpoint;
}

pcl::PointXYZRGB  worldToImagePlane( pcl::PointXYZRGB p){
    float fx_d=577.30;
    float fy_d=579.41;
    float cx_d=319.5;
    float cy_d=239.5;
    pcl::PointXYZRGB out;
    out.x = (int)((fx_d*p.x/p.z)+cx_d);
    out.y = (int)(cy_d-(fy_d*p.y/p.z));
    out.z = p.z;
    return out;
}


void computePointcloud()
{
    cloud.clear();
    int cols=_image.cols;
    int rows=_image.rows;
    cv::Point p;
    float v;
    pcl::PointXYZRGB pxyz;
    for (int i=0;i<cols;i++){
        for(int j=0;j<rows;j++){


            p.x=i;
            p.y=j;
            v=((float)_image.at<ushort>(p));

            if(v!=0){
                cv::Vec3f localPoint(p.x,p.y,(float)v);
                cv::Vec3f worldPoint=localToWorld(localPoint);

                pxyz.x=worldPoint[0];
                pxyz.y=worldPoint[1];
                pxyz.z=worldPoint[2];

                uint8_t r = 0, g = 0, b = 255;    // Example: Red color
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                pxyz.rgb = *reinterpret_cast<float*>(&rgb);
                cloud.push_back(pxyz);
            }
            else{
                cv::Vec3f localPoint(p.x,p.y,(float)v);
                cv::Vec3f worldPoint=localToWorld(localPoint);

                pxyz.x=worldPoint[0];
                pxyz.y=worldPoint[1];
                pxyz.z=NAN;

                uint8_t r = 0, g = 0, b = 255;    // Example: Red color
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                pxyz.rgb = *reinterpret_cast<float*>(&rgb);
                cloud.push_back(pxyz);
            }
        }
    }


}

void computeCalibrationMatrix(){



    if(1){
        pcl::PointXYZRGB point;
        pcl::PointXYZRGB projected_point;

        for(unsigned int i=0; i<cloud.size();i++){
            point=cloud.at(i);
            projected_point=errorCloud.at(i);
            Eigen::Vector3f diff = point.getVector3fMap() - projected_point.getVector3fMap();
            double measuredDistance= diff.norm();

            pcl::PointXYZRGB localPoint = worldToImagePlane(point);
            if(localPoint.x>0 && localPoint.y>0 && validPoints.at(i)){

                if(point.z<projected_point.z)
                    multiplier->cell(localPoint.y,
                                    localPoint.x,
                                    localPoint.z,(measuredDistance+localPoint.z)/localPoint.z);
                if(point.z>projected_point.z)
                    multiplier->cell(localPoint.y,
                                    localPoint.x,
                                    localPoint.z,(localPoint.z-measuredDistance)/localPoint.z);

                multiplier->increment(localPoint.y,
                                     localPoint.x,
                                     localPoint.z);

            }



        }
    }


}


void computerCenterPlane(){
    if(centerCloud.size()>10){
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (50);

        seg.setInputCloud (centerCloud.makeShared ());
        seg.segment (*inliers, *coefficients);

        planeCoefficient[0]=coefficients->values[0];
        planeCoefficient[1]=coefficients->values[1];
        planeCoefficient[2]=coefficients->values[2];
        planeCoefficient[3]=coefficients->values[3];


        float dot = -1*planeCoefficient[2];
        if(dot<0){
            planeCoefficient[0]*=-1;
            planeCoefficient[1]*=-1;
            planeCoefficient[2]*=-1;
            //            planeCoefficient[3]*=-1;
        }

        pcl::compute3DCentroid<pcl::PointXYZRGB>(centerCloud,planeCentroid);
//        std::cout << "centroid at "<< planeCentroid<<std::endl;
    }
}


void voxelize(float leaf){
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud.makeShared());
    float _voxelLeaf =leaf;
    sor.setLeafSize ((float)_voxelLeaf, (float)_voxelLeaf, (float)_voxelLeaf);
    sor.filter (cloud);
}


void computeNormals(){
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    //        pcl::PointCloud<pcl::PointXYZ>::Ptr p(cloud.makeShared());
    ne.setInputCloud (cloud.makeShared());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    cloud_normals.clear();
    ne.setRadiusSearch (100); //MALCOM
    ne.compute (cloud_normals);
}

void computeCenterSquareCloud(){
    centerCloud.clear();
    pcl::PointXYZRGB pix;
    for(unsigned int i=0; i<cloud.size();i++){
        pix=cloud.at(i);
        pcl::PointXYZRGB local = worldToImagePlane(pix);
        if(local.x>240 && local.x<400 && local.y>140 && local.y<340){
            centerCloud.push_back(pix);
            pix.z=0;
            uint8_t r,g,b;

            r= 0, g = 0, b = 255;


            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            cloud.at(i).rgb = *reinterpret_cast<float*>(&rgb);
        }
    }
}


void pointrejection(float errorRange){
    Eigen::Vector3f nReference(planeCoefficient[0],
                               planeCoefficient[1],
                               planeCoefficient[2]);
    pcl::Normal n;
    validPoints.clear();
    for(unsigned int i=0; i<cloud.size();i++){
        n=cloud_normals.at(i);
        Eigen::Vector3f n1(n.normal_x,n.normal_y,n.normal_z);
        Eigen::Vector3f cross=n1.cross(nReference);
        if(cross.norm()>errorRange){ //NORMAL REJECTION
            validPoints.push_back(false);
        }
        else{
            validPoints.push_back(true);
        }


    }
}


void computeErrorPerPoint(){
    errorCloud.clear();
    pcl::PointXYZRGB p;
    pcl::PointXYZRGB pp;
    Eigen::Vector3f res;
    for(unsigned int i=0; i<cloud.size();i++){
        p=cloud.at(i);
        pcl::geometry::project( Eigen::Vector3f(p.x,p.y,p.z),
                                Eigen::Vector3f(planeCentroid(0),planeCentroid(1),planeCentroid(2)),
                                Eigen::Vector3f(planeCoefficient(0),planeCoefficient(1),planeCoefficient(2)),
                                res);
        pp.x=res(0);
        pp.y=res(1);
        pp.z=res(2);
        errorCloud.push_back(pp);
    }
}
