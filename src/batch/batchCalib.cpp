#include <getopt.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "../shared/CalibrationMatrix.h"
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


void processImage(cv::Mat& image){
    std::cout <<image.cols<<" "<<image.rows<<std::endl;
}




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

int main (int argc, char **argv)
{
    //MANAGING COMMAND LINE ARGUMENTS
    int c;
    while (1)
    {
        static struct option long_options[] =
        {
            /* These options set a flag. */
            {"input", required_argument,       0, 'i'},
            {"output", required_argument,       0, 'o'},
            {0, 0, 0, 0}
        };
        /* getopt_long stores the option index here. */
        int option_index = 0;

        c = getopt_long (argc, argv, "i:o:",long_options, &option_index);

        /* Detect the end of the options. */
        if (c == -1)
            break;

        switch (c)
        {
        case 0:
            /* If this option set a flag, do nothing else now. */
            if (long_options[option_index].flag != 0)
                break;
            printf ("option %s", long_options[option_index].name);
            if (optarg)
                printf (" with arg %s", optarg);
            printf ("\n");
            break;

        case 'i':
            printf ("-\tinput directory `%s'\n", optarg);
            break;

        case 'o':
            printf ("-\toutput directory `%s'\n", optarg);

            if (stat(optarg, &sta) == -1) {
                printf ("-\tcreating  directory `%s'\n", optarg);
                mkdir(optarg, 0700);
            }
            else{
                printf ("-\tdirectory `%s' already exists\n", optarg);
            }
            break;

        default:
            abort ();
        }
    }


    /* Print any remaining command line arguments (not options). */
    if (optind < argc)
    {
        printf ("non-option ARGV-elements: ");
        while (optind < argc)
            printf ("%s ", argv[optind++]);
        putchar ('\n');
    }
    //-----------------------------------------------


    //LISTING DIRECTORY
    //-----------------------------------------------
    struct dirent **eps;
    int n;
    n = scandir (".", &eps, pgmfilter, alphasort);
    //END
    if (n >= 0)
    {
        int cnt;
        for (cnt = 0; cnt < n; ++cnt){
            std::cout<< "opening "<<eps[cnt]->d_name<<std::endl;
            cv::Mat image = cv::imread(eps[cnt]->d_name,CV_LOAD_IMAGE_UNCHANGED);
            processImage(image);
            std::cout<< "done!"<<std::endl;
        }
    }
    //-----------------------------------------------
    exit (0);
}
