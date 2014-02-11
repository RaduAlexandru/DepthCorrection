#ifndef KM_M
#define KM_M

#include <map>
#include <iostream>
#include <math.h>
#include <opencv/cv.hpp>
#include <iostream>
#include <fstream>
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

        _data = new float**[layers];
        for (int i=0; i<layers; i++){
            _data[i] = new float* [rows];
            for (int j=0; j< rows; j++){
                _data[i][j]=new float[cols];
                for (int k=0; k< cols; k++){
                    _data[i][j][k]=1.0f;

                }
            }
        }

        _hits = new float**[layers];
        for (int i=0; i<layers; i++){
            _hits[i] = new float* [rows];
            for (int j=0; j< rows; j++){
                _hits[i][j]=new float[cols];
                for (int k=0; k< cols; k++){
                    _hits[i][j][k]=1.0f;

                }
            }
        }


    }

    float cell(int r, int c, int d){
        if(useKernel==0)
            return _data[d>>depthPow][r>>tilePow][c>>tilePow]/_hits[d>>depthPow][r>>tilePow][c>>tilePow];
        //            return _data[d>>depthPow][r>>tilePow][c>>tilePow];
        else if(useKernel){

        }
    }

    void cell(int r, int c, int d, float mply){

        if(useKernel==0){
            _data[d>>depthPow][r>>tilePow][c>>tilePow]+=mply;
            //             _data[d>>depthPow][r>>tilePow][c>>tilePow]=mply;
            //            increment(r,c,d);
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

    //    void initToZeros(){
    //        for (int i=0; i<layers; i++){

    //            for (int j=0; j< rows; j++){

    //                for (int k=0; k< cols; k++){
    //                    _data[i][j][k]=0.0f;

    //                }
    //            }
    //        }
    //    }

    //    void initToOnes(){
    //        for (int i=0; i<layers; i++){

    //            for (int j=0; j< rows; j++){

    //                for (int k=0; k< cols; k++){
    //                    _data[i][j][k]=1.0f;

    //                }
    //            }
    //        }
    //    }

    //    void average(CalibrationMatrix c){
    //        for (int i=0; i<layers; i++){

    //            for (int j=0; j< rows; j++){

    //                for (int k=0; k< cols; k++){
    //                    _data[i][j][k]=_data[i][j][k]/c._data[i][j][k];

    //                }
    //            }
    //        }
    //    }

    void serialize(char* filename){
        std::cout<<"opening handle...";
        std::ofstream writer(filename);
        std::cout<<"saving...";
        std::cout<< layers << " "<<rows << " "<<cols<<std::endl;
        writer<<layers<<" "<<rows<<" "<<cols<<" "<<std::endl;

        for (int i=0; i<layers; i++){

            for (int j=0; j< rows; j++){

                for (int k=0; k< cols; k++){
                    writer<<_data[i][j][k]/_hits[i][j][k]<<" ";
                    //                    std::cout<<".";
                }

            }
            writer<<std::endl;
        }
        writer.close();
        std::cout<<"done"<<std::endl;

    }

    void deserialize(char* filename){
        std::ifstream myfile (filename);
        int layers;
        int rows;
        int cols;

        myfile >> layers >> rows >> cols;
        std::cout << " from file "<<layers<<" " <<rows<<" "<<cols;
        std::cout.flush();
        if(layers!= this->layers && rows != this->rows && cols!=this->cols){
            std::cout << "file data incosistent"<<std::endl;
        }
        else{
            for (int i=0; i<layers; i++){

                for (int j=0; j< rows; j++){

                    for (int k=0; k< cols; k++){
                        myfile>>_data[i][j][k];
                        std::cout << _data[i][j][k]<<std::endl;
                    }

                }

            }
        }
        myfile.close();
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
    float*** _data;
    float*** _hits;
};




#endif
