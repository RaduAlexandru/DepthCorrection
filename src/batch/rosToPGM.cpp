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
using namespace cv_bridge;

static int verbose_flag;
static int i=0;
void callback(const sensor_msgs::ImageConstPtr &imgPtr){
    cv_bridge::CvImagePtr cv_ptr;
    std::cout << ".";
    char buff[50];
    sprintf(buff,"%05d.pgm",i);
    cv_ptr = toCvCopy(imgPtr,sensor_msgs::image_encodings::MONO16);
    cv::imwrite(buff,cv_ptr->image);
    i++;
    std::cout << ":";
}

int main (int argc, char **argv)
{
    std::cout<<"INIT"<<std::endl;
    ros::init(argc, argv, "xtionBatch");
    ros::NodeHandle n("xtionBatch");
    std::cout<<"ROSNODE INIT"<<std::endl;
    ros::Subscriber s = n.subscribe("/camera/depth/image_raw", 5, callback);
    ros::spin();
//    int c;

//    while (1)
//    {
//        static struct option long_options[] =
//        {
//            /* These options set a flag. */
//            {"verbose", no_argument,       &verbose_flag, 1},
//            {"brief",   no_argument,       &verbose_flag, 0},
//            /* These options don't set a flag.
//                      We distinguish them by their indices. */
//            {"add",     no_argument,       0, 'a'},
//            {"append",  no_argument,       0, 'b'},
//            {"delete",  required_argument, 0, 'd'},
//            {"create",  required_argument, 0, 'c'},
//            {"file",    required_argument, 0, 'f'},
//            {0, 0, 0, 0}
//        };
//        /* getopt_long stores the option index here. */
//        int option_index = 0;

//        c = getopt_long (argc, argv, "abc:d:f:",
//                         long_options, &option_index);

//        /* Detect the end of the options. */
//        if (c == -1)
//            break;

//        switch (c)
//        {
//        case 0:
//            /* If this option set a flag, do nothing else now. */
//            if (long_options[option_index].flag != 0)
//                break;
//            printf ("option %s", long_options[option_index].name);
//            if (optarg)
//                printf (" with arg %s", optarg);
//            printf ("\n");
//            break;

//        case 'a':
//            puts ("option -a\n");
//            break;

//        case 'b':
//            puts ("option -b\n");
//            break;

//        case 'c':
//            printf ("option -c with value `%s'\n", optarg);
//            break;

//        case 'd':
//            printf ("option -d with value `%s'\n", optarg);
//            break;

//        case 'f':
//            printf ("option -f with value `%s'\n", optarg);
//            break;

//        case '?':
//            /* getopt_long already printed an error message. */
//            break;

//        default:
//            abort ();
//        }
//    }

//    /* Instead of reporting ‘--verbose’
//          and ‘--brief’ as they are encountered,
//          we report the final status resulting from them. */
//    if (verbose_flag)
//        puts ("verbose flag is set");

//    /* Print any remaining command line arguments (not options). */
//    if (optind < argc)
//    {
//        printf ("non-option ARGV-elements: ");
//        while (optind < argc)
//            printf ("%s ", argv[optind++]);
//        putchar ('\n');
//    }

    exit (0);
}
