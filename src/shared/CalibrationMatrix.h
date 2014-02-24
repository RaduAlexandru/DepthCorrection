#ifndef KM_M
#define KM_M

#include <map>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <fstream>

using namespace std;

class CalibrationMatrix{
public:
    CalibrationMatrix(int rows, int cols, int maxDepth, int tileSize, int depthRes);
    CalibrationMatrix(char* filename);
    float cell(int r, int c, int d);
    void cell(int r, int c, int d, float mply);
    void increment(int r, int c, int d);
    void serialize(char* filename);
    void deserialize(char* filename);
    CalibrationMatrix* downsample(int dxy, int dd);
    void clear();
    void worldToMap(int& ir, int& ic, int& id, int r, int c, int d);
    void mapToWorld(int& r, int& c, int& d,int ir, int ic, int id);
    void dumpSensorImages();
    void dumpCovariance();
    void syncToFloat();
    float getFloat(int r, int c, int d);
    int maxDepth;
    int tileSize;
    int tilePow;
    int depthRes;
    int depthPow;
    int rows;
    int cols;
    int layers;
    bool useKernel;
    float*** _data;
    float*** _staticData;
    float*** _hits;
    float*** _covariance;
};




#endif
