#include "../shared/CalibrationMatrix.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dirent.h>
#include <getopt.h>
#include <string>

using namespace std;

string inputfileame;
string outputfilename="out.txt";
bool hasToDump=false;
int imagescale=1;
int depthscale=1;

int main(int argc, char **argv)
{


    //MANAGING COMMAND LINE ARGUMENTS
    int c;
    if(argc<=1){
        printf("Options:\n");
        printf("--input -i string inputfilename\n");
        printf("--output -o string outputfilename\n");
        printf("--dumpImages -d bool \n");
        printf("--depthscale -j int \n");
        printf("--imagescale -k int \n");
        exit(1);
    }
    while (1)
    {
        static struct option long_options[] =
        {
            /* These options set a flag. */
            {"input",        required_argument,       0, 'i'},
            {"output",       required_argument,       0, 'o'},
            {"imagescale",   required_argument,   0, 'k'},
            {"depthscale",   required_argument,   0, 'j'},
            {"dumpImages",   no_argument,             0, 'd'},
            {0, 0, 0, 0}
        };
        /* getopt_long stores the option index here. */
        int option_index = 0;

        c = getopt_long (argc, argv, "i:o:d",long_options, &option_index);

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
            printf ("-\tinput filename `%s'\n", optarg);
            inputfileame=optarg;
            break;

        case 'o':
            printf ("-\toutput filename `%s'\n", optarg);
            outputfilename=optarg;
            break;

        case 'd':
            printf ("-\tsensor images will be dumped\n");
            hasToDump=true;
            break;
        case 'k':
            printf ("-\timage scaling by `%d'\n", atoi(optarg));
            imagescale=atoi(optarg);
            break;

        case 'j':
            printf ("-\tdepth scaling by %d\n", atoi(optarg));
            depthscale=atoi(optarg);
            break;
        default:
            abort ();
        }
    }


    CalibrationMatrix _multiplier((char*)inputfileame.c_str());

    std::cerr << "matrix loaded" << std::endl;
    std::cout << "scaling image by "<<imagescale<< "  depth scaling by "<<depthscale<<std::endl;
    CalibrationMatrix* scaledMultiplier = _multiplier.downsample(imagescale,depthscale);
    CalibrationMatrix& multiplier = *scaledMultiplier;
    multiplier.serialize((char*)outputfilename.c_str());
    if(hasToDump){
        multiplier.dumpSensorImages();
    }


    return 0;
}
