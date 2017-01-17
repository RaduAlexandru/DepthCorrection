#include "mySubscriber.h"
#include <opencv2/opencv.hpp>
#include <pcl/features/integral_image_normal.h>


#define SQUARE_SIZE_X 40
#define SQUARE_SIZE_Y 40
#define PLANCE_DISTANCE_THRESHOLD 0.001

//#define F_X 532.7886771534169
//#define F_Y 532.5948115815787
//#define C_X 468.3525646507127
//#define C_Y 265.31061972652395

MySubscriber::MySubscriber(FancyViewer* v) :
    m_shutdown_required(false),
    m_thread(&MySubscriber::spin, *this),
    m_processing_counter(0),
    m_first_cloud(true)
{

    m_viewer=v;
    m_applyCorrection=false;
    m_recordData=false;
    m_voxelLeaf=10;
    m_normalRejection=0.7f;
    m_show_planeModelInliers=true;
    m_computeRefenceDistance=false;
    m_refenceDistance=0;
}

MySubscriber::~MySubscriber(){
    m_shutdown_required = true;
    m_thread.join();
}



void MySubscriber::callback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg){


    std::cout << "processing " << m_processing_counter << std::endl;
    m_processing_counter++;


    if(!this->m_queue->status())
        {
            this->m_queue->lock();


            //when we receive the first point cloud we will not the size to make out multiplier
            if (m_first_cloud){
                m_multiplier = new CalibrationMatrix (cloud_msg->height,cloud_msg->width,12,1,1);
                m_first_cloud=false;
            }


            //read projection matrix
            m_fx= cam_info_msg->P[0];
            m_fy= cam_info_msg->P[5];
            m_cx= cam_info_msg->P[2];
            m_cy= cam_info_msg->P[6];


            //std::cout << "callback, started processing" << std::endl;


            computePointcloud(cloud_msg);
//            voxelize();
            computeCenterSquareCloud();
            computerCenterPlane();
            computeNormals();
            pointrejection();
            computeErrorPerPoint();
            computeCalibrationMatrix();
            calibratePointCloudWithMultipliers();





            std::cout << "plane centroid is " <<   m_planeCentroid(0);


            QueuePayload payload;
            payload.cloud=this->m_cloud;
            payload.planeCentroid=this->m_planeCentroid;
            payload.planeCoefficient=this->m_planeCoefficient;
            payload.errorCloud=this->m_errorCloud;
            payload.correctCloud=this->m_correctCloud;
            payload.normals= this->m_cloud_normals;
            payload.validPoints=this->m_validPoints;
            this->m_queue->data=payload;
            this->m_viewer->data=payload;
            this->m_queue->unlock();
        }






//    //dirty workaround to the encoding issue
//	if (imgPtr->encoding == "16UC1"){
//                        sensor_msgs::Image img;
//                        img.header = imgPtr->header;
//                        img.height = imgPtr->height;
//                        img.width = imgPtr->width;
//                        img.is_bigendian = imgPtr->is_bigendian;
//                        img.step = imgPtr->step;
//                        img.data = imgPtr->data;
//                        img.encoding = "mono16";

//                        _image = cv_bridge::toCvCopy(img, "mono16");
//                    }




//    //_image = cv_bridge::toCvCopy(imgPtr, "mono8");
//    if(!this->queue->status())
//    {
//        this->queue->lock();

//        //std::cout << "callback, started processing" << std::endl;


//        computePointcloud();
//        voxelize();
//        computeCenterSquareCloud();
//        //computeCenterCloud();
//        computerCenterPlane();
//        computeNormals();
//        pointrejection();
//        computeErrorPerPoint();
//        computeCalibrationMatrix();
//        calibratePointCloudWithMultipliers();

//        QueuePayload payload;
//        payload.cloud=this->cloud;
//        payload.planeCentroid=this->planeCentroid;
//        payload.planeCoefficient=this->planeCoefficient;
//        payload.errorCloud=this->errorCloud;
//        payload.correctCloud=this->correctCloud;
//        payload.normals= this->cloud_normals;
//        payload.validPoints=this->validPoints;
//        this->queue->data=payload;
//        this->_viewer->data=payload;
//        this->queue->unlock();
//    }
}

void MySubscriber::spin() {
    ros::Rate loop(10);
    sleep(1);
    while ( ros::ok() && !m_shutdown_required ) {
        ros::spinOnce();
        loop.sleep();
    }
}

//VOXELIZE COMPUTATION
void MySubscriber::voxelize(){
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (m_cloud.makeShared());
    float _voxelLeaf =m_voxelLeaf;
    sor.setLeafSize ((float)_voxelLeaf, (float)_voxelLeaf, (float)_voxelLeaf);
    sor.filter (m_cloud);
}


//POINTCLOUD COMPUTATION
void MySubscriber::computePointcloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    m_cloud.clear();
    pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*cloud_msg,*temp_cloud);
    pcl::fromPCLPointCloud2(*temp_cloud,m_cloud);

    //make it bigger
    for(int i=0; i< m_cloud.size();i++){
//                cloud.points[i].x=cloud.points[i].x*100;
//                cloud.points[i].y=cloud.points[i].y*100;
//                cloud.points[i].z=cloud.points[i].z*100;
//        if (!isnan(cloud.points[i].z) )
//            std::cout << " " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
    }

    //std::cout << "cloud height is " << cloud.height << std::endl;
    //std::cout << "cloud size is " << cloud.size() << std::endl;


}
void MySubscriber::computeCenterSquareCloud(){
    m_centerCloud.clear();
    pcl::PointXYZRGB pix; 


    int center_x= m_cloud.width/2;
    int center_y= m_cloud.height/2;
    
    int border_x_low= center_x - SQUARE_SIZE_X;
    int border_x_high = center_x + SQUARE_SIZE_X;
    int border_y_low = center_y - SQUARE_SIZE_Y;
    int border_y_high = center_y + SQUARE_SIZE_Y;

    for(unsigned int i=0; i<m_cloud.size();i++){
        pix=m_cloud.at(i);
        pcl::PointXYZRGB local = worldToImagePlane(pix);
        if(local.x> border_x_low  && local.x< border_x_high && local.y> border_y_low && local.y< border_y_high){
            m_centerCloud.push_back(pix);
            pix.z=0;
            uint8_t r,g,b;
            if(m_show_planeModelInliers){
                r= 0, g = 255, b = 0;
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                m_cloud.at(i).rgb = *reinterpret_cast<float*>(&rgb);
            }
//            if(!m_show_planeModelInliers){
//                r= 0, g = 0, b = 255;
//            }


        }
    }
}


void MySubscriber::computeCenterCloud(){
    m_centerCloud.clear();
    pcl::PointXYZRGB pix;
    m_validNormalRange=300.0f;
    for(unsigned int i=0; i<m_cloud.size();i++){
        pix=m_cloud.at(i);

        if(sqrt(pix.x*pix.x+pix.y*pix.y)<=m_validNormalRange){
            m_centerCloud.push_back(pix);
            pix.z=0;
            uint8_t r,g,b;
            if(m_show_planeModelInliers){
                r= 0, g = 255, b = 0;
            }
            if(!m_show_planeModelInliers){
                r= 0, g = 0, b = 255;
            }

            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            m_cloud.at(i).rgb = *reinterpret_cast<float*>(&rgb);
        }
    }

}


void MySubscriber::computerCenterPlane(){
    if(m_centerCloud.size()>10){
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (PLANCE_DISTANCE_THRESHOLD);

        seg.setInputCloud (m_centerCloud.makeShared ());
        seg.segment (*inliers, *coefficients);

        m_planeCoefficient[0]=coefficients->values[0];
        m_planeCoefficient[1]=coefficients->values[1];
        m_planeCoefficient[2]=coefficients->values[2];
        m_planeCoefficient[3]=coefficients->values[3];


        float dot = -1*m_planeCoefficient[2];
        if(dot<0){
            m_planeCoefficient[0]*=-1;
            m_planeCoefficient[1]*=-1;
            m_planeCoefficient[2]*=-1;
            //            planeCoefficient[3]*=-1;
        }

        std::cout << "center cloud is " << m_centerCloud.size() << std::endl;
        std::cout << "inliers is " << inliers->indices.size() << std::endl;


        pcl::compute3DCentroid<pcl::PointXYZRGB>(m_centerCloud,m_planeCentroid);

    }
}


void MySubscriber::computeErrorPerPoint(){
    m_errorCloud.clear();
    pcl::PointXYZRGB p;
    pcl::PointXYZRGB pp;
    Eigen::Vector3f res;
    for(unsigned int i=0; i<m_cloud.size();i++){
        p=m_cloud.at(i);
        pcl::geometry::project( Eigen::Vector3f(p.x,p.y,p.z),
                                Eigen::Vector3f(m_planeCentroid(0),m_planeCentroid(1),m_planeCentroid(2)),
                                Eigen::Vector3f(m_planeCoefficient(0),m_planeCoefficient(1),m_planeCoefficient(2)),
                                res);
        pp.x=res(0);
        pp.y=res(1);
        pp.z=res(2);
        m_errorCloud.push_back(pp);
    }
}


//WORLD REFERENCE FRAME POINT CLOUD COMPUTATION
cv::Vec3f MySubscriber::localToWorld(cv::Vec3f localpoint)
{
    /*float fx_d=577.30;
    float fy_d=579.41;
    float cx_d=319.5;
    float cy_d=239.5;*/
    cv::Vec3f worldpoint;
    worldpoint[0]=(localpoint[0]-m_cx)*localpoint[2]/m_fx;
    worldpoint[1]=(localpoint[1]-m_cy)*localpoint[2]/m_fy;
    worldpoint[2]=localpoint[2];
    return worldpoint;
}

pcl::PointXYZRGB  MySubscriber::worldToImagePlane( pcl::PointXYZRGB p){
    /*float fx_d=577.30;
    float fy_d=579.41;
    float cx_d=319.5;
    float cy_d=239.5;*/
    pcl::PointXYZRGB out;
    out.x = (int)((m_fx*p.x/p.z)+m_cx);
    out.y = (int)(m_cy-(m_fy*p.y/p.z));
    out.z = p.z;
    return out;
}

void MySubscriber::computeCalibrationMatrix(){



    if(m_recordData){
        pcl::PointXYZRGB point;
        pcl::PointXYZRGB projected_point;

        for(unsigned int i=0; i<m_cloud.size();i++){
            point=m_cloud.at(i);
            projected_point=m_errorCloud.at(i);
            Eigen::Vector3f diff = point.getVector3fMap() - projected_point.getVector3fMap();
            double measuredDistance= diff.norm();

            pcl::PointXYZRGB localPoint = worldToImagePlane(point);
            if(localPoint.x>0 && localPoint.y>0 && m_validPoints.at(i)){

                if(point.z<projected_point.z){
                    std::cout << "z is smaller accesing cell at " << localPoint.y << " " << localPoint.x << " " << (measuredDistance+localPoint.z)/localPoint.z << std::endl;
                    m_multiplier->cell(localPoint.y,
                                    localPoint.x,
                                    localPoint.z,(measuredDistance+localPoint.z)/localPoint.z);
                }

                if(point.z>projected_point.z){
                    std::cout << "z is bigger accesing cell at " << localPoint.y << " " << localPoint.x << " " << (measuredDistance+localPoint.z)/localPoint.z << std::endl;
                    m_multiplier->cell(localPoint.y,
                                    localPoint.x,
                                    localPoint.z,(localPoint.z-measuredDistance)/localPoint.z);
                }


                m_multiplier->increment(localPoint.y,
                                     localPoint.x,
                                     localPoint.z);


                //                std::cout<<  multiplier.cell(localPoint.y,
                //                                             localPoint.x,
                //                                             localPoint.z)<<std::endl;
                //                hits.increment(localPoint.y,
                //                               localPoint.x,
                //                               localPoint.z);
            }



        }
    }


}

void MySubscriber::calibratePointCloudWithMultipliers(){

    if(m_applyCorrection || 1){
        pcl::PointXYZRGB point;
        pcl::PointXYZRGB projected_point;
        float averageError=0;
        float averageDistance=0;
        for(unsigned int i=0; i<m_cloud.size();i++){
            point=m_cloud.at(i);
            pcl::PointXYZRGB localPoint = worldToImagePlane(point);
            cv::Vec3f local(localPoint.x,localPoint.y,localPoint.z);
            cv::Vec3f tst =localToWorld(local);


            if(localPoint.x>0 && localPoint.y>0){

                if(m_applyCorrection){
                    //                  cloud.at(i).z*= multiplier.cell(localPoint.y,localPoint.x,localPoint.z)/hits.cell(localPoint.y,localPoint.x,localPoint.z);
                    //                  cloud.at(i).z*= multiplier.cell(localPoint.y,localPoint.x,localPoint.z);
                    localPoint.z*=m_multiplier->cell(localPoint.y,localPoint.x,localPoint.z);
//                  std::cout<< "MPLIER: "<<multiplier.cell(localPoint.y,localPoint.x,localPoint.z)<<std::endl;
                    cv::Vec3f local(localPoint.x,localPoint.y,localPoint.z);
                    cv::Vec3f tst =localToWorld(local);
                    m_cloud.at(i).x=tst[0];
                    m_cloud.at(i).y=-tst[1];
                    m_cloud.at(i).z=tst[2];
                    averageDistance+=m_cloud.at(i).z;
                    averageError+=pow(m_cloud.at(i).z-(float)m_refenceDistance,2);
                    //                                std::cout <<"\t readen "<< cloud.at(i).z << " has to be "<< refenceDistance<< " incremental error is " << averageError<<std::endl;
                }
                else{
                    averageError+=pow(m_cloud.at(i).z-(float)m_refenceDistance,2);
                    averageDistance+=m_cloud.at(i).z;
                    //                                std::cout <<"\t readen "<< cloud.at(i).z << " has to be "<< refenceDistance<< " incremental error is " << averageError<<std::endl;
                }
            }


        }
        if(m_computeRefenceDistance){
            std::cout <<"average distance "<<averageDistance/m_cloud.size() <<" average error "<<sqrt(averageError)/m_cloud.size()<<"mm" << std::endl;
        }
    }
}

void MySubscriber::computeNormals(){
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(10.0f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(m_cloud.makeShared());
    m_cloud_normals.clear();
    ne.compute(m_cloud_normals);
}


void MySubscriber::pointrejection(){
    Eigen::Vector3f nReference(m_planeCoefficient[0],
                               m_planeCoefficient[1],
                               m_planeCoefficient[2]);
    pcl::Normal n;
    m_validPoints.clear();
    for(unsigned int i=0; i<m_cloud.size();i++){
        n=m_cloud_normals.at(i);
        Eigen::Vector3f n1(n.normal_x,n.normal_y,n.normal_z);
        Eigen::Vector3f cross=n1.cross(nReference);
        if(cross.norm()>m_normalRejection){
            m_validPoints.push_back(false);
        }
        else{
            m_validPoints.push_back(true);
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


