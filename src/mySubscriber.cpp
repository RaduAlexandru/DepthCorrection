#include "mySubscriber.h"
#include <opencv2/opencv.hpp>
MySubscriber::MySubscriber(FancyViewer* v) : shutdown_required(false),thread(&MySubscriber::spin, *this),
    multiplier(480,640,10000,4,8),
    hits(480,640,10000,4,8)
{

    this->_viewer=v;
    hits.initToZeros();
    applyCorrection=false;
    recordData=false;
}

MySubscriber::~MySubscriber(){
    shutdown_required = true;
    thread.join();
}

void MySubscriber::callback(const sensor_msgs::ImageConstPtr &imgPtr){

    _image = cv_bridge::toCvCopy(imgPtr, "mono16");
    if(!this->queue->status())
    {
        this->queue->lock();


        computePointcloud();
        voxelize();

        computeCenterCloud();
        computerCenterPlane();
        computeErrorPerPoint();
        computeCalibrationMatrix();
        calibratePointCloudWithMultipliers();

        QueuePayload payload;
        payload.cloud=this->cloud;
        payload.planeCentroid=this->planeCentroid;
        payload.planeCoefficient=this->planeCoefficient;
        payload.errorCloud=this->errorCloud;
        payload.correctCloud=this->correctCloud;
        this->queue->data=payload;
        this->_viewer->data=payload;
        this->queue->unlock();
    }
}

void MySubscriber::spin() {
    ros::Rate loop(10);
    sleep(1);
    while ( ros::ok() && !shutdown_required ) {
        ros::spinOnce();
        loop.sleep();
    }
}

//VOXELIZE COMPUTATION
void MySubscriber::voxelize(){
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (this->cloud.makeShared());
    float _voxelLeaf =3.0f;
    sor.setLeafSize ((float)_voxelLeaf, (float)_voxelLeaf, (float)_voxelLeaf);
    sor.filter (cloud);
}


//POINTCLOUD COMPUTATION
void MySubscriber::computePointcloud()
{
    cloud.clear();
    int cols=this->_image->image.cols;
    int rows=this->_image->image.rows;
    cv::Point p;
    ushort v;
    pcl::PointXYZRGB pxyz;
    for (int i=0;i<cols;i++){
        for(int j=0;j<rows;j++){


            p.x=i;
            p.y=j;
            v=((float)this->_image->image.at<ushort>(p))*0.1f;

            if(v!=0){
                cv::Vec3f localPoint(p.x,p.y,(float)v);
                cv::Vec3f worldPoint=localToWorld(localPoint);

                pxyz.x=worldPoint[0];
                pxyz.y=worldPoint[1];
                pxyz.z=worldPoint[2];

                uint8_t r = 0, g = 255, b = 0;    // Example: Red color
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                pxyz.rgb = *reinterpret_cast<float*>(&rgb);
                cloud.push_back(pxyz);
            }
        }
    }


}


void MySubscriber::computeCenterCloud(){
    this->centerCloud.clear();
    pcl::PointXYZRGB pix;
    _validNormalRange=30.0f;
    for(unsigned int i=0; i<cloud.size();i++){
        pix=cloud.at(i);

        if(sqrt(pix.x*pix.x+pix.y*pix.y)<=_validNormalRange){
            this->centerCloud.push_back(pix);
            pix.z=0;
            uint8_t r = 255, g = 255, b = 255;    // Example: Red color
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            cloud.at(i).rgb = *reinterpret_cast<float*>(&rgb);
        }
    }

}


void MySubscriber::computerCenterPlane(){
    if(this->centerCloud.size()>10){
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

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

    }
}


void MySubscriber::computeErrorPerPoint(){
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


//WORLD REFERENCE FRAME POINT CLOUD COMPUTATION
cv::Vec3f MySubscriber::localToWorld(cv::Vec3f localpoint)
{
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

pcl::PointXYZRGB  MySubscriber::worldToImagePlane( pcl::PointXYZRGB p){
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

void MySubscriber::computeCalibrationMatrix(){



    if(recordData){
        pcl::PointXYZRGB point;
        pcl::PointXYZRGB projected_point;

        for(unsigned int i=0; i<cloud.size();i++){
            point=cloud.at(i);
            projected_point=errorCloud.at(i);
            Eigen::Vector3f diff = point.getVector3fMap() - projected_point.getVector3fMap();
            double measuredDistance= diff.norm();

            pcl::PointXYZRGB localPoint = worldToImagePlane(point);
            if(localPoint.x>0 && localPoint.y>0){

                if(point.z<projected_point.z)
                    multiplier.cell(localPoint.y,
                                    localPoint.x,
                                    localPoint.z,(measuredDistance+localPoint.z)/localPoint.z);
                if(point.z>projected_point.z)
                    multiplier.cell(localPoint.y,
                                    localPoint.x,
                                    localPoint.z,(localPoint.z-measuredDistance)/localPoint.z);

            }

            hits.increment(localPoint.y,
                           localPoint.x,
                           localPoint.z);

        }
    }


}

void MySubscriber::calibratePointCloudWithMultipliers(){

    if(applyCorrection){
        pcl::PointXYZRGB point;
        pcl::PointXYZRGB projected_point;

        for(unsigned int i=0; i<cloud.size();i++){
            point=cloud.at(i);
            pcl::PointXYZRGB localPoint = worldToImagePlane(point);
            if(localPoint.x>0 && localPoint.y>0){

                if(applyCorrection)
                    cloud.at(i).z*= multiplier.cell(localPoint.y,localPoint.x,localPoint.z);
            }


        }
    }


    //    //    std::cout<<" REF "<< q.planeCentroid.transpose();
    //    cv::Mat errorImage(480,640,CV_32FC1);
    //    cv::Mat error(480,640,CV_8UC1);
    //    errorImage=cv::Mat::zeros(480,640,CV_32FC1);
    //    cv::Point p;
    //    for(unsigned int i=0; i<cloud.size();i++){
    //        pcl::PointXYZRGB localPoint = worldToImagePlane(cloud.at(i));
    //        p.y=localPoint.y;
    //        p.x=localPoint.x;
    //        float v=(float)localPoint.z;
    //        if(p.x>0 && p.y>0){
    //            errorImage.at<float>(p)=multiplier.cell(p.y,p.x,localPoint.z);
    ////            std::cout << " pre: "<<cloud.at(i).z;
    ////              cloud.at(i).z*=0*multiplier.cell(p.y,p.x,cloud.at(i).z);
    ////            std::cout << " post: "<<cloud.at(i).z<<std::endl;
    //        }
    //    }
    ////    cv::flip(errorImage,errorImage,0);
    ////    double min;
    ////    double max;
    ////    cv::minMaxLoc(errorImage,&min,&max);
    ////    errorImage.convertTo(error,CV_8UC1, 255 / (max-min), -min);
    ////    cv::Mat dest;
    ////    cv::applyColorMap(error,dest,cv::COLORMAP_OCEAN);
    ////    cv::imshow("colormap",dest);

}
