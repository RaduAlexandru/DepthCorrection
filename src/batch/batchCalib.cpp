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
#include "batchCalib.h"
#include <string>

using namespace std;

//--------------------------------------------------------
int pgmfilter(const struct dirent *dir){
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

static int one (const struct dirent *unused){
    return 1;
}

//--------------------------------------------------------

//needed to create a directory
struct stat sta = {0};
float voxelLeaf=20;
float normalrange=0.8;
string inputdir="";
int tilesize=1;
int depthres=1;
int maxdepth=10000;
int frameskip=1;
int iocchi=0;
string output="out.txt";

int main (int argc, char **argv)
{
    //MANAGING COMMAND LINE ARGUMENTS
    int c;
    if(argc<=1){
        printf("Options:\n");
        printf("--voxeleaf -v float inputfilename\n");
        printf("--normalrange -n float outputfilename\n");
        printf("--inputdir -i string \n");
        printf("--tilesize -t int \n");
        printf("--depthres -d int \n");
        printf("--maxdepth -m int \n");
        printf("--output -o string \n");
        printf("--frameskip -f string \n");
        printf("--iocchilog -l  \n");
        exit(1);
    }
    while (1)
    {
        static struct option long_options[] =
        {
            /* These options set a flag. */
            {"voxeleaf", required_argument,          0, 'v'},
            {"normalrange", required_argument,       0, 'n'},
            {"inputdir", required_argument,          0, 'i'},
            {"tilesize", required_argument,          0, 't'},
            {"depthres", required_argument,          0, 'd'},
            {"maxdepth", required_argument,          0, 'm'},
            {"output", required_argument,          0, 'o'},
            {"iocchilog", no_argument,          0, 'l'},
            {"frameskip", required_argument,          0, 'f'},
            {0, 0, 0, 0}
        };
        /* getopt_long stores the option index here. */
        int option_index = 0;

        c = getopt_long (argc, argv, "v:n:i:t:d:m:o:f:l",long_options, &option_index);

        /* Detect the end of the options. */
        if (c == -1)
            break;

        switch (c)
        {


        case 'v':
            printf ("-\tvoxel leaf `%s'\n", optarg);
            voxelLeaf=atof(optarg);
            break;

        case 'n':
            printf ("-\tnormal range `%s'\n", optarg);
            normalrange=atof(optarg);
            break;

        case 'i':
            printf ("-\tinput directory `%s'\n", optarg);
            inputdir=optarg;
            break;

        case 't':
            printf ("-\ttilesize `%s'\n", optarg);
            tilesize=atoi(optarg);
            break;

        case 'd':
            printf ("-\tdepthres `%s'\n", optarg);
            depthres=atoi(optarg);
            break;

        case 'o':
            printf ("-\toutput filename `%s'\n", optarg);
            output=optarg;
            break;

        case 'm':
            printf ("-\tmaxdepth `%s'\n", optarg);
            maxdepth=atoi(optarg);
            break;

        case 'f':
            printf ("-\tframeskip `%s'\n", optarg);
            frameskip=atoi(optarg);
            break;

        case 'l':
            printf ("-\tiocchilog `%s'\n", optarg);
            iocchi=true;
            break;

        default:
            abort ();
        }
    }



    //LISTING DIRECTORY
    //-----------------------------------------------
    struct dirent **eps;
    int n;
    string scanDir="./";
    scanDir.append(inputdir);
    n = scandir (scanDir.c_str(), &eps, pgmfilter, alphasort);
    //END
    multiplier = new CalibrationMatrix(480,640,maxdepth,tilesize,depthres);

    if (n >= 0)
    {
        int cnt;
        for (cnt = 0; cnt < n; cnt+=frameskip){
            string path;
            path.append(scanDir);
            path.append(eps[cnt]->d_name);
            std::cout<< "["<<cnt<<"/"<<n << "] opening "<<path<<std::endl;
            cv::Mat image = cv::imread(path,CV_LOAD_IMAGE_UNCHANGED);
            processImage(image, voxelLeaf, normalrange,iocchi);
            std::cout<< "done!"<<std::endl;

        }
    }

    std::cout<< "serialization!"<<std::endl;
    multiplier->serialize((char*)output.c_str());
    multiplier->dumpSensorImages();

    //-----------------------------------------------
    exit (0);
}
