#ifndef KM_M
#define KM_M

#include <map>
#include <iostream>
#include <math.h>
#include <opencv/cv.hpp>
//-------------------------------------------------i--------------
class CalibrationMatrix{
public:
    CalibrationMatrix(int rows, int cols, int maxDepth, int tileSize, int depthRes){
        useKernel=false;
        this->_rows=rows;
        this->_cols=cols;
        this->maxDepth=maxDepth;
        this->tileSize=tileSize;
        this->tilePow=log2(this->tileSize);
        this->depthRes=depthRes;
        this->depthPow=log2(this->depthRes);
        this->rows=_rows/tileSize;
        this->cols=_cols/tileSize;
        this->layers=this->maxDepth/this->depthRes;

        _data = new double**[layers];
        for (int i=0; i<layers; i++){
            _data[i] = new double* [rows];
            for (int j=0; j< rows; j++){
                _data[i][j]=new double[cols];
                for (int k=0; k< cols; k++){
                    _data[i][j][k]=1.0f;

                }
            }
        }

        _hits = new double**[layers];
        for (int i=0; i<layers; i++){
            _hits[i] = new double* [rows];
            for (int j=0; j< rows; j++){
                _hits[i][j]=new double[cols];
                for (int k=0; k< cols; k++){
                    _hits[i][j][k]=0.0f;

                }
            }
        }


    }

    float cell(int r, int c, int d){
        if(useKernel==0)
//            return _data[d>>depthPow][r>>tilePow][c>>tilePow]/_hits[d>>depthPow][r>>tilePow][c>>tilePow];
            return _data[d>>depthPow][r>>tilePow][c>>tilePow];
        else if(useKernel){

        }
    }

    void cell(int r, int c, int d, float mply){

        if(useKernel==0){
//            _data[d>>depthPow][r>>tilePow][c>>tilePow]+=mply;
             _data[d>>depthPow][r>>tilePow][c>>tilePow]=mply;
            increment(r,c,d);
//            std::cout << "d: "<<_data[d>>depthPow][r>>tilePow][c>>tilePow];
//            std::cout<<" h: "<<_hits[d>>depthPow][r>>tilePow][c>>tilePow];
//            std::cout<<" mplier: "<<cell(r,c,d)<<std::endl;
        }
        else if(useKernel){

        }
    }

    void increment(int r, int c, int d){

        _hits[d>>depthPow][r>>tilePow][c>>tilePow]+=1.0f;
    }

    void worldToMap(int& ir, int& ic, int& id, int r, int c, int d){
        ir=r>>tilePow;
        ic=c>>tilePow;
        id=d>>depthPow;
    }

    void mapToWorld(int& r, int& c, int& d,int ir, int ic, int id){
        r=ir<<tilePow;
        c=ic<<tilePow;
        d=id<<depthPow;
    }

    void initToZeros(){
        for (int i=0; i<layers; i++){

            for (int j=0; j< rows; j++){

                for (int k=0; k< cols; k++){
                    _data[i][j][k]=0.0f;

                }
            }
        }
    }

    void average(CalibrationMatrix c){
        for (int i=0; i<layers; i++){

            for (int j=0; j< rows; j++){

                for (int k=0; k< cols; k++){
                    _data[i][j][k]=_data[i][j][k]/c._data[i][j][k];

                }
            }
        }
    }



    int _rows;
    int _cols;
    int maxDepth;
    int tileSize;
    int tilePow;
    int depthRes;
    int depthPow;
    int rows;
    int cols;
    int layers;
    bool useKernel;
    double*** _data;
    double*** _hits;
};




#endif
