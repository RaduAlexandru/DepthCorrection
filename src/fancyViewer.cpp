#include "fancyViewer.h"
//#define UPSIZE 1


void FancyViewer::animate()
{

    //float vAmbientLightBright[4] = {0.5f, 0.5f, 0.5f, 1.0f};
    GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_diffuse);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);

}

void FancyViewer::draw()
{

    if(data.cloud.size()){
        drawPointcloud();
        drawCentralNormal(data.planeCentroid,data.planeCoefficient);
//        drawNormals();
    }


}

FancyViewer::FancyViewer(QWidget *parent) : QGLViewer(parent)
{
    printf("-> Viewer initialized\n");
    rejectPoints=true;
}

void FancyViewer::changeBgColor(){
    this->setBackgroundColor(QColor(br,bg,bb));
}

void FancyViewer::init()
{
    br=bb=bg=255;
    setGridIsDrawn(true);
    startAnimation();
    setSceneRadius(1);
    camera()->setType(qglviewer::Camera::ORTHOGRAPHIC);
    camera()->showEntireScene();
    this->camera()->setZNearCoefficient(0.00000001);
    this->camera()->setZClippingCoefficient(1000000.0);
    this->toggleFPSIsDisplayed();
    this->setBackgroundColor(QColor(br,bg,bb));
    pointSize=2;
}

void FancyViewer::drawCentralNormal(Eigen::Vector4f p, Eigen::Vector4f c){

    glPushMatrix();
    glRotatef(180,1,0,0);
    glLineWidth(5.0f);
    glBegin(GL_LINES);

    glColor3f(1,1,0);
    glVertex3f(p[0],p[1],p[2]);

    Eigen::Vector3f t(c[0],c[1],c[2]);
    t.normalize();

    glVertex3f(p[0]+t[0]*2.0,p[1]+t[1]*2.0,p[2]+t[2]*2.0);

    glEnd();
    glPopMatrix();


}

void FancyViewer::drawNormals(){
    //    glPushMatrix();
    //    glRotatef(180,1,0,0);

    //    for(unsigned int i=0; i<data.normals.size();i++){

    //        pcl::Normal n = data.normals.at(i);

    //    }
    //    glPopMatrix();
}

void FancyViewer::drawPointcloud(){

    glPointSize((float)pointSize);
    glPushMatrix();
    glRotatef(180,1,0,0);

    pcl::PointXYZRGB p;

    glBegin(GL_POINTS);

    for(unsigned int i=0; i<data.cloud.size();i++){
                p=data.cloud.at(i);
//                p.x=p.x*UPSIZE;
//                p.y=p.y*UPSIZE;
//                p.z=p.z*UPSIZE;

                float r = p.r/256.0;
                float g = p.g/256.0;
                float b = p.b/256.0;

                glColor3f(r,g,b);
                glVertex3f(p.x,p.y,p.z);



    }
    glEnd();
    glPopMatrix();



//    glPointSize((float)pointSize);
//    glPushMatrix();
//    glRotatef(180,1,0,0);
//    pcl::PointXYZRGB p;
//    pcl::PointXYZRGB q;
//    pcl::Normal n;
//    if(data.cloud.size() == data.errorCloud.size()) {


//        for(unsigned int i=0; i<data.cloud.size();i++){
//            p=data.cloud.at(i);
//            p.x=p.x*300;
//            p.y=p.y*300;
//            p.z=p.z*300;
//            q=data.errorCloud.at(i);
//            n = data.normals.at(i);

//            //std::cout << "normal is " << n.normal_x << " " << n.normal_y << " " <<  n.normal_z << std::endl;

//            uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
////            uint8_t r = (rgb >> 16) & 0x0000ff;
////            uint8_t g = (rgb >> 8)  & 0x0000ff;
////            uint8_t b = (rgb)       & 0x0000ff;

//            float r = p.r/256.0;
//            float g = p.g/256.0;
//            float b = p.b/256.0;

//            glLineWidth(2.0f);

//            glBegin(GL_POINTS);
////            glColor3f(1.0,
////                      0.0,
////                      0.0);

//            glColor3f(r,
//                      g,
//                      b);

//            glVertex3f(p.x,p.y,p.z);
//            glEnd();
//            if(rejectPoints==false){
//                glBegin(GL_LINES);
//                glColor3f(0,
//                          0,
//                          0);
//                glVertex3f(p.x,p.y,p.z);


//                glColor3f(0,
//                          0,
//                          1);


//                Eigen::Vector3f n1(n.normal_x,n.normal_y,n.normal_z);
//                Eigen::Vector3f nReference(data.planeCoefficient[0],
//                                           data.planeCoefficient[1],
//                                           data.planeCoefficient[2]);
//                Eigen::Vector3f cross=n1.cross(nReference);
//                if(data.validPoints.at(i)==false){
//                    glColor3f(1,0,0);
//                }
//                glVertex3f(400*n.normal_x+p.x,
//                           400*n.normal_y+p.y,
//                           400*n.normal_z+p.z);
//                glEnd();
//            }


//        }
//    }
//    glPopMatrix();
}
