#include "driver.h"
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <sensor_msgs/image_encodings.h>


void  xtionDriver::callback(const sensor_msgs::ImageConstPtr &imgPtr){
    cvImageFromROS =cv_bridge::toCvCopy(imgPtr);

    //   cv::Mat image;
    //   cv::Mat original;

    cvImageFromROS->image.copyTo(image);
    cvImageFromROS->image.copyTo(original);

    int cols=image.cols;
    int rows=image.rows;
    //cv::Point p;
    //ushort v;
    for (int i=0;i<cols;i++){
        for(int j=0;j<rows;j++){
            p.x=i;
            p.y=j;
            v=((float)image.at<ushort>(p));
            //v*=multiplier.cell(p.y,p.x,v);
            v*=multiplier.getFloat(p.y,p.x,v);
            image.at<ushort>(p)=(ushort)v;
        }
    }

    //cv_bridge::CvImage out_msg;
    out_msg.header   = imgPtr->header;
    out_msg.encoding = "mono16";
    out_msg.image    = image;
    pub.publish(out_msg.toImageMsg());

    //cv_bridge::CvImage out_msg_DIFF;
    out_msg_DIFF.header   = imgPtr->header;
    out_msg_DIFF.encoding = "mono16";
    out_msg_DIFF.image    = (image-original);
    pubDIFF.publish(out_msg_DIFF.toImageMsg());


}

void xtionDriver::spin() {
    ros::Rate loop(100);
    sleep(1);
    while ( ros::ok() && !shutdown_required ) {
        ros::spinOnce();
        loop.sleep();
    }
}

