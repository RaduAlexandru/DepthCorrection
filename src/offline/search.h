#include <unistd.h>
#include <iostream>
#include <dirent.h>
#include <vector>

std::vector<std::string> results;				// holds search results

// recursive search algorithm
void search(std::string curr_directory, std::string extension){
    DIR* dir_point = opendir(curr_directory.c_str());
    dirent* entry = readdir(dir_point);
    while (entry){									// if !entry then end of directory
        if (entry->d_type == DT_DIR){				// if entry is a directory
            std::string fname = entry->d_name;
            if (fname != "." && fname != "..")
                search(entry->d_name, extension);	// search through it
        }
        else if (entry->d_type == DT_REG){		// if entry is a regular file
            std::string fname = entry->d_name;	// filename
                                                // if filename's last characters are extension
            if (fname.find(extension, (fname.length() - extension.length())) != std::string::npos)
                results.push_back(fname);		// add filename to results vector
        }
        entry = readdir(dir_point);
    }
    return;
}
