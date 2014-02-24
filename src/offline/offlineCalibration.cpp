#include "../shared/CalibrationMatrix.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dirent.h>


int pgmfilter(const struct dirent *dir)
// post: returns 1/true if name of dir ends in .mp3
{
    const char *s = dir->d_name;
    int len = strlen(s) - 4;    // index of start of . in .mp3
    if(len >= 0)
    {
        if (strncmp(s + len, ".pgm", 4) == 0)
        {
            return 1;
        }
    }
    return 0;
}

static int one (const struct dirent *unused)
{
    return 1;
}


int main(int argc, char **argv)
{
    CalibrationMatrix  multiplier  ("prova.txt");

    struct dirent **eps;
    int n;
    n = scandir ("./images/", &eps, pgmfilter, alphasort);
    if (n >= 0)
    {
        int cnt;
        for (cnt = 0; cnt < n; ++cnt){
            std::cout<< "opening "<<eps[cnt]->d_name<<std::endl;
            cv::Mat image;
            std::string filename="./images/";
            filename.append(eps[cnt]->d_name);
            std::cout<<"OPENING:\t "<<filename<<std::endl;
            image = cv::imread(filename,CV_LOAD_IMAGE_UNCHANGED);


            int cols=image.cols;
            int rows=image.rows;
            cv::Point p;
            ushort v;
            for (int i=0;i<cols;i++){
                for(int j=0;j<rows;j++){
                    p.x=i;
                    p.y=j;
                    v=((float)image.at<ushort>(p));

//                   if(multiplier.cell(p.y,p.x,v/10)!=1.0f)
//                        std::cout << "was "<<v;
                    v*=multiplier.cell(p.y,p.x,v);
//                    if(multiplier.cell(p.y,p.x,v/10)!=1.0f)
//                        std::cout << " is "<<v<<std::endl;

                    image.at<ushort>(p)=(ushort)v;
                }
            }
            std::string outDir=("./images/calibrated/");
            outDir.append(eps[cnt]->d_name);
            cv::imwrite(outDir,image);
            std::cout<< "saved "<<outDir<<std::endl;
        }
    }
    else
        perror ("Couldn't open the directory");


    return 0;
}
