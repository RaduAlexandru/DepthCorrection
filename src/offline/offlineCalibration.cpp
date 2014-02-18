#include "../shared/CalibrationMatrix.h"
#include "search.h"

int main(int argc, char **argv)
{

    CalibrationMatrix  multiplier  (480,640,5000,2,2500);
    multiplier.deserialize("prova.txt");
    multiplier.serialize("prova2.txt");
    //multiplier.dumpSensorImages();
    multiplier.dumpCovariance();
    std::string curr_directory = get_current_dir_name();
    search(curr_directory, ".txt");

    // output results
    if (results.size()){
        std::cout << results.size() << " files were found:" << std::endl;
        for (unsigned int i = 0; i < results.size(); ++i)	// used unsigned to appease compiler warnings
            std::cout << "- \t" <<  results[i] << std::endl;
    }
    else{
        std::cout << "No files ending in '" << "extension" << "' were found." << std::endl;
    }
    return 0;
}
