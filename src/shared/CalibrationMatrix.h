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
    CalibrationMatrix(int rows, int cols, float maxDepth, int tileSize, float depthRes);
    CalibrationMatrix(char* filename);
    float cell(int r, int c, float d);
    void cell(int r, int c, float d, float mply);
    void increment(int r, int c, float d);
    void serialize(char* filename);
    void deserialize(char* filename);
    CalibrationMatrix* downsample(int dxy, int dd);
    void clear();
    void worldToMap(int& ir, int& ic, int& id, int r, int c, int d);
    void mapToWorld(int& r, int& c, int& d,int ir, int ic, int id);
    void dumpSensorImages();
    void dumpCovariance();
    void syncToFloat();
    void dumpMe();
    float  growTop(int row, int col, int dep);
    float  growBottom(int row, int col, int dep);
    float  growLeft(int row, int col, int dep);
    float  growRight(int row, int col, int dep);

    void getStats();
    float cellNN(int k,int r, int c, int d);
    float getFloat(int r, int c, int d);
    float maxDepth;
    int tileSize;
    int tilePow;
    float depthRes;
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
