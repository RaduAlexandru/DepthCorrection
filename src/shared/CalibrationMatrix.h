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
    CalibrationMatrix(int rows, int cols, int maxDepth, int tileSize, int depthRes){
        useKernel=false;
        this->maxDepth=maxDepth;
        this->tileSize=tileSize;
        this->tilePow=log2(this->tileSize);
        this->depthRes=depthRes;
        this->depthPow=log2(this->depthRes);
        this->rows=rows/tileSize;
        this->cols=cols/tileSize;
        this->layers=this->maxDepth/this->depthRes;


	
	cerr << "tailsaiz: " << this->tileSize  << endl; 
        cerr << "allocating" << layers << "x" << this->rows << "x" << this->cols << " things" << endl; 
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


        _covariance = new float**[layers];
        for (int i=0; i<layers; i++){
            _covariance[i] = new float* [rows];
            for (int j=0; j< rows; j++){
                _covariance[i][j]=new float[cols];
                for (int k=0; k< cols; k++){
                    _covariance[i][j][k]=1.0f;

                }
            }
        }


    }

    float cell(int r, int c, int d){
      if(useKernel==0){
	int h = _hits[d>>depthPow][r>>tilePow][c>>tilePow];
	if (h){
	  return _data[d>>depthPow][r>>tilePow][c>>tilePow]/h;
	}
	return 1;
        //            return _data[d>>depthPow][r>>tilePow][c>>tilePow];
      }
  }

    void cell(int r, int c, int d, float mply){

        if(useKernel==0){
            _data[d>>depthPow][r>>tilePow][c>>tilePow]+=mply;
            _covariance[d>>depthPow][r>>tilePow][c>>tilePow]+=mply*mply;

        }
        else if(useKernel){

            //            for( int i =-1;i<2;i++){
            //                for( int j =-1;j<2;j++){
            //                    for( int k =-1;k<2;k++){
            //                        _data[(d+i)>>depthPow][(r+j)>>tilePow][(c+k)>>tilePow]+=mply;
            //                        _hits[(d+i)>>depthPow][(r+j)>>tilePow][(c+k)>>tilePow]+=1.0f;
            //                    }
            //                }
            //            }

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





    //-----------------------------------------------------------------------------------SAVE TO FILE
    void serialize(char* filename){
        std::cout<<"opening handle...";
        std::ofstream writer(filename);
        std::cout<<"saving...";
        std::cout<< layers << " "<<rows << " "<<cols<<std::endl;
        std::cout.flush();
        writer<<layers<<" "<<rows<<" "<<cols<<" "<<std::endl;

        for (int i=0; i<this->layers; i++){

            for (int j=0; j< this->rows; j++){

                for (int k=0; k< this->cols; k++){
                    //writer<<_data[i][j][k]<< " " << _hits[i][j][k]<<" " << sqrt((_covariance[i][j][k]/_hits[i][j][k]) -_data[i][j][k])<<" ";
                    writer<<_data[i][j][k]<< " " << _hits[i][j][k]<<" " << sqrt(((_data[i][j][k]*_data[i][j][k])/_hits[i][j][k])-(_data[i][j][k]/_hits[i][j][k])*(_data[i][j][k]/_hits[i][j][k]))<<" ";
                }

            }
            writer<<std::endl;
        }
        writer.close();
        std::cout<<"done"<<std::endl;

    }
    //-----------------------------------------------------------------------------------LOAD FROM FILE
  void deserialize(char* filename){
        std::ifstream myfile (filename);
        int layers;
        int rows;
        int cols;

        myfile >> layers >> rows >> cols;
        std::cout << " from file "<<layers<<" " <<rows<<" "<<cols << std::endl;
        std::cout.flush();
        if(layers!= this->layers && rows != this->rows && cols!=this->cols){
            std::cout << "file data incosistent"<<std::endl;
            std::cout << "should be "<<layers << " "<<rows <<" " << cols<<std::endl;
        }
        else{
            for (int i=0; i<layers; i++){

                for (int j=0; j< rows; j++){

                    for (int k=0; k< cols; k++){
                        myfile>>_data[i][j][k];
                        myfile>>_hits[i][j][k];
                        myfile>>_covariance[i][j][k];
                    }

                }

            }
        }
        myfile.close();
    }

  CalibrationMatrix* downsample(int dxy, int dd){
    CalibrationMatrix* downsampled = new CalibrationMatrix(rows*tileSize, cols*tileSize, maxDepth+depthRes*dd, dxy*tileSize, depthRes*dd);

    std::cerr << "allocated" << std::endl;

    std::cerr << "clearing" << std::endl;
    for (int i=0; i<downsampled->layers; i++){
      for (int j=0; j< downsampled->rows; j++){
	for (int k=0; k< downsampled->cols; k++){
	  downsampled->_data[i][j][k] = 0;
	  downsampled->_hits[i][j][k] = 0;
	  downsampled->_covariance[i][j][k] = 0;
	}
      }
    }

    std::cerr << "filling" << std::endl;

    for (int i=0; i<layers; i++){
      for (int j=0; j< rows; j++){
	for (int k=0; k< cols; k++){
	  downsampled->_data[i/dd][j/dxy][k/dxy] += _data[i][j][k];
	  downsampled->_hits[i/dd][j/dxy][k/dxy] += _hits[i][j][k];
	  downsampled->_covariance[i/dd][j/dxy][k/dxy] = _covariance[i][j][k];
	}

      }

    }
    return downsampled;
  }

    void dumpSensorImages(){
        for (int i=0; i<layers; i++){
            cv::Mat errorImage(rows,cols,CV_32FC1);
            cv::Mat error(rows,cols,CV_8UC1);
            errorImage=cv::Mat::zeros(rows,cols,CV_32FC1);
            cv::Point p;
            for (int j=0; j< rows; j++){

                for (int k=0; k< cols; k++){
                    p.y=j;
                    p.x=k;
                    float v= _data[i][j][k]/_hits[i][j][k];


                    errorImage.at<float>(p)=(v-1)*3000+127;
                }

            }
            cv::flip(errorImage,errorImage,0);
            double min;
            double max;

            //errorImage.convertTo(error,CV_8UC1, 255.0/(max - min), -min * 255.0/(max - min));
            //errorImage.convertTo(error,CV_8UC1, 255 / (max-min), -min);
            errorImage.convertTo(error,CV_8UC1);
            cv::minMaxIdx(error,&min,&max);
            std::cout << "MIN: "<<min << "MAX: "<<max<<std::endl;
            //cv::convertScaleAbs(errorImage, error, 255 / max);
            cv::Mat dest;
            char filename[50];
            for(int colormap =0;colormap<1;colormap++){
                cv::applyColorMap(error,dest,colormap);
                sprintf(filename,"COLORMAP[%d]layer_%d.pgm",colormap,i);
                cv::imwrite(filename,dest);std::cout<< "saved "<<filename<<std::endl;
                //std::cout<< "saved "<<filename<<std::endl;
            }

        }
    }


    void dumpCovariance(){
        for (int i=0; i<layers; i++){
            cv::Mat errorImage(rows,cols,CV_32FC1);
            cv::Mat error(rows,cols,CV_8UC1);
            errorImage=cv::Mat::zeros(rows,cols,CV_32FC1);
            cv::Point p;
            for (int j=0; j< rows; j++){

                for (int k=0; k< cols; k++){
                    p.y=j;
                    p.x=k;
                    float v= _covariance[i][j][k];


                    errorImage.at<float>(p)=v;
                }

            }
            cv::flip(errorImage,errorImage,0);
            double min;
            double max;
            cv::minMaxIdx(errorImage,&min,&max);
            std::cout << "MIN: "<<min << "MAX: "<<max<<std::endl;
            //errorImage.convertTo(error,CV_8UC1, 255.0/(max - min), -min * 255.0/(max - min));
            errorImage.convertTo(error,CV_8UC1, 128);
            //cv::convertScaleAbs(errorImage, error, 255 / max);
            cv::Mat dest;
            char filename[50];
            for(int colormap =0;colormap<12;colormap++){
                cv::applyColorMap(error,dest,colormap);
                sprintf(filename,"COLORMAP[%d]layer_%d.pgm",colormap,i);
                cv::imwrite(filename,dest);std::cout<< "saved "<<filename<<std::endl;
                //std::cout<< "saved "<<filename<<std::endl;
            }

        }
    }

    //---------------------------------------------------------------------------------------ATTRIBUTES
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
    float*** _covariance;
};




#endif
