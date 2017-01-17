#include "CalibrationMatrix.h"


float interpolate ( float input , float input_start, float input_end, float output_start, float output_end){

  float output;
  output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start);

  return output;

}


CalibrationMatrix::CalibrationMatrix(int rows, int cols, float maxDepth, int tileSize, float depthRes){
//    useKernel=false;
//    this->maxDepth=maxDepth;
//    this->tileSize=tileSize;
//    this->tilePow=log2(this->tileSize);
//    this->depthRes=depthRes;
//    this->depthPow=log2(this->depthRes);
//    this->rows=rows/tileSize;
//    this->cols=cols/tileSize;
//    this->layers=this->maxDepth/this->depthRes;


    this->maxDepth=maxDepth;
    this->tileSize=tileSize;
    this->tilePow=log2(this->tileSize);
    this->depthRes=depthRes;
    this->depthPow=log2(this->depthRes);
    this->rows=rows/tileSize;
    this->cols=cols/tileSize;
    this->layers=std::ceil(this->maxDepth/this->depthRes);



    cerr << "tailsaiz: " << this->tileSize  << endl;
    cerr << "allocating" << layers << "x" << this->rows << "x" << this->cols << " cells" << endl;


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

    _staticData = new float**[layers];
    for (int i=0; i<layers; i++){
        _staticData[i] = new float* [rows];
        for (int j=0; j< rows; j++){
            _staticData[i][j]=new float[cols];
            for (int k=0; k< cols; k++){
                _staticData[i][j][k]=1.0f;

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


void CalibrationMatrix::clear(){


    for (int i=0; i<layers; i++){

        for (int j=0; j< rows; j++){

            //            for (int k=0; k< cols; k++){
            //                delete(_data[i][j][k]);

            //            }
            delete(_data[i][j]);
        }
        delete(_data[i]);
    }
    delete(_data);


    for (int i=0; i<layers; i++){

        for (int j=0; j< rows; j++){

            //            for (int k=0; k< cols; k++){
            //                delete(_staticData[i][j][k]);

            //            }
            delete(_staticData[i][j]);
        }
        delete(_staticData[i]);
    }
    delete(_staticData);


    for (int i=0; i<layers; i++){

        for (int j=0; j< rows; j++){

            //            for (int k=0; k< cols; k++){
            //                delete(_hits[i][j][k]);

            //            }
            delete(_hits[i][j]);
        }
        delete(_hits[i]);
    }
    delete(_hits);


    for (int i=0; i<layers; i++){

        for (int j=0; j< rows; j++){

            //            for (int k=0; k< cols; k++){
            //                delete(_covariance[i][j][k]);

            //            }
            delete(_covariance[i][j]);
        }
        delete(_covariance[i]);
    }
    delete(_covariance);

}

float CalibrationMatrix::cell(int r, int c, float d){
    if(d>this->layers*this->depthRes){
       printf("[WARNING] requested depth out of calib range\n");
        return 1;
    }

    int z=std::floor (d/this->depthRes);
    int y=std::floor (r/this->tileSize);
    int x=std::floor (c/this->tileSize);


    //return _data[d>>depthPow][r>>tilePow][c>>tilePow]/_hits[d>>depthPow][r>>tilePow][c>>tilePow];
    return _data[z][y][x]/_hits[z][y][x];
}

void CalibrationMatrix::cell(int r, int c, float d, float mply){

    //std::cout << "CalibrationMatrix::cell raw " << r  << " " << c << " " << d << std::endl;

//    int z=d>>depthPow;
//    int y=r>>tilePow;
//    int x=c>>tilePow;
//    std::cout << "CalibrationMatrix::cell data << " << y << " " << x << " " << z << std::endl;

      int z=std::floor (d/this->depthRes);
      int y=std::floor (r/this->tileSize);
      int x=std::floor (c/this->tileSize);


    //std::cout << "CalibrationMatrix::cell input << " << d << " " << r << " " << c << std::endl;
    //std::cout << "CalibrationMatrix::cell data acces << " << z << " " << y << " " << x << std::endl << std::endl;

    _data[z][y][x]+=mply;
    _covariance[z][y][x]+=mply*mply;

}

void CalibrationMatrix::syncToFloat(){

    for (int i=0; i<layers; i++){

        for (int j=0; j< rows; j++){

            for (int k=0; k< cols; k++){
                _staticData[i][j][k]=_data[i][j][k]/_hits[i][j][k];

            }
        }
    }
}

float CalibrationMatrix::getFloat(int r, int c, int d){
    return _staticData[d>>depthPow][r>>tilePow][c>>tilePow];
}

void CalibrationMatrix::increment(int r, int c, float d){

     //std::cout << "CalibrationMatrix::increment" << std::endl;

    int z=std::floor (d/this->depthRes);
    int y=std::floor (r/this->tileSize);
    int x=std::floor (c/this->tileSize);

    _hits[z][y][x]+=1.0f;

}

void CalibrationMatrix::getStats(){
    int unos=0;
    for (int i=0; i<layers; i++){

        for (int j=0; j< rows; j++){

            for (int k=0; k< cols; k++){

                if(_hits[i][j][k]==1){
                    unos++;
                    //                    if(this->cellNN(1,j,k,i)!=1)
                    //                        std::cout << std::endl<<"data was 1, now is "<<this->cellNN(1,j,k,i)<<std::endl;
                    //                    else
                    //                        std::cout<<"miss.";

                }

            }
        }
    }
    std::cout << "############ missing data on "<<unos<<"/"<<layers*rows*cols<<" " <<(float)((float)unos/((float)(layers*rows*cols))) <<std::endl;
}

float  CalibrationMatrix::growLeft(int row, int col, int dep){
    if(_hits[dep][row][col]==1.0f && row<(rows-1))
    {
        growLeft(++row,col,dep);
    }
    return _data[dep][row][col]/_hits[dep][row][col];
}

float  CalibrationMatrix::growRight(int row, int col, int dep){
    if(_hits[dep][row][col]==1.0f && row>0)
    {
        growLeft(--row,col,dep);
    }
    return _data[dep][row][col]/_hits[dep][row][col];
}


float  CalibrationMatrix::growTop(int row, int col, int dep){
    if(_hits[dep][row][col]==1.0f && col>0)
    {
        growLeft(row,--col,dep);
    }
    return _data[dep][row][col]/_hits[dep][row][col];
}


float  CalibrationMatrix::growBottom(int row, int col, int dep){
    if(_hits[dep][row][col]==1.0f && col<(cols-1))
    {
        growLeft(row,++col,dep);
    }
    return _data[dep][row][col]/_hits[dep][row][col];
}

float CalibrationMatrix::cellNN(int k,int row, int col, int dep){
    float v=0;
    v+=growLeft(row,col,dep);
    v+=growRight(row,col,dep);
    v+=growTop(row,col,dep);
    v+=growBottom(row,col,dep);

    return v/4;

}



void CalibrationMatrix::worldToMap(int& ir, int& ic, int& id, int r, int c, int d){
    ir=r>>tilePow;
    ic=c>>tilePow;
    id=d>>depthPow;
}

void CalibrationMatrix::mapToWorld(int& r, int& c, int& d,int ir, int ic, int id){
    r=ir<<tilePow;
    c=ic<<tilePow;
    d=id<<depthPow;
}





//-----------------------------------------------------------------------------------SAVE TO FILE
void CalibrationMatrix::serialize(char* filename){
    std::cout<<"opening handle...";
    std::ofstream writer(filename);
    std::cout<<"saving...";
    std::cout<< layers << " "<<rows << " "<<cols<<" "<<depthRes<<std::endl;
    std::cout.flush();
    writer<<layers<<" "<<rows<<" "<<cols<<" "<<" "<<depthRes<< std::endl;

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

CalibrationMatrix::CalibrationMatrix(char* filename){
    std::ifstream myfile (filename);
    int layers;
    int rows;
    int cols;
    int depthres;

    myfile >> layers >> rows >> cols >> depthres;
    std::cout << "allocating from file "<<layers<<" " <<rows<<" "<<cols <<" "<<depthres << std::endl;
    std::cout.flush();


    this->maxDepth=depthres*layers;
    this->tileSize=960/cols;
    this->tilePow=log2(this->tileSize);
    this->depthRes=depthres;
    this->depthPow=log2(this->depthRes);
    this->rows=rows;
    this->cols=cols;
    this->layers=layers;

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

    _staticData = new float**[layers];
    for (int i=0; i<layers; i++){
        _staticData[i] = new float* [rows];
        for (int j=0; j< rows; j++){
            _staticData[i][j]=new float[cols];
            for (int k=0; k< cols; k++){
                _staticData[i][j][k]=1.0f;

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

void CalibrationMatrix::deserialize(char* filename){
    std::ifstream myfile (filename);
    int layers;
    int rows;
    int cols;
    int buff;
    myfile >> layers >> rows >> cols >>buff;
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

CalibrationMatrix* CalibrationMatrix::downsample(int dxy, int dd){
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

void CalibrationMatrix::dumpSensorImages(){


    //find the minimum (disregarding the 0) multiplier and the maximum multiplier
    //the multipler with 0 should correspond to a value of 127 in the error image

    float min=std::numeric_limits<float>::max();
    float max=std::numeric_limits<float>::lowest();

    for (int i=0; i<layers; i++){
        for (int j=0; j< rows; j++){
            for (int k=0; k< cols; k++){
                float v= _data[i][j][k]/_hits[i][j][k];
                if(_hits[i][j][k]==1)v=0;

                if (v < min && v!=0) min=v;
                if (v > max && v!=0) max=v;

                }

            }

      }

    std::cout << "min max is" << min << " " << max << std::endl;

    //values from 1 to max should be mapped to range 127 to 255
    //values from min to 1 should be mapped to range 0 to 127
    //then we can choose a colormap like the COLORMAP_JET that assigns red for the points that are pushed and blue for the ones that are pulled towards the camera



    for (int i=0; i<layers; i++){

        cv::Mat errorImage(rows,cols,CV_8UC1);
        errorImage=cv::Mat::ones(rows,cols,CV_8UC1);
        errorImage=errorImage*127; //Initialize it at 127 to indicate that no multiplier is applied (no pushing nor pullingS)

        for (int j=0; j< rows; j++){
            for (int k=0; k< cols; k++){
                float v= _data[i][j][k]/_hits[i][j][k];
                if(_hits[i][j][k]==1)v=0;

                int interpolated_value;

                if (v>1 && v!=0){
                    interpolated_value= interpolate (v, 1,max, 127,255);
                    errorImage.at<uchar>(j,k)=interpolated_value;
                }
                if (v<1 && v!=0){
                    interpolated_value= interpolate (v, min,1, 0,127);
                    errorImage.at<uchar>(j,k)=interpolated_value;
                }



             }

         }

        cv::Mat dest;
        char filename[50];
        cv::flip(errorImage,errorImage,0);
        cv::applyColorMap(errorImage,dest,cv::COLORMAP_JET);

        //all the pixels that are at 127 mean that they do not have any multipler so we assing them a color black so it doesnt interfere with the other colors
        cv::Vec3b black;
        black[0]=0;
        black[1]=0;
        black[2]=0;
        for (int j=0; j< rows; j++){
            for (int k=0; k< cols; k++){
                if(errorImage.at<uchar>(j,k)==127){
                    dest.at<cv::Vec3b>(j,k)=black;
                }

             }

        }


        sprintf(filename,"layer_%d.pgm",i);
        cv::imwrite(filename,dest);
        std::cout << "saved image at layer: " << i <<std::endl;

      }

}


void CalibrationMatrix::dumpMe(){
    //    std::cout<<"DDDDDDDUUUUUUUUUUMMMMMMMMPPPPPPPPPP";

    //    std::cout<<"saving...";
    //    std::cout<< layers << " "<<rows << " "<<cols<<" "<<depthRes<<std::endl;
    //    std::cout.flush();


    //    for (int i=0; i<this->layers; i++){
    //        char buff[100];
    //        sprintf(buff,"layer_%d.txt",i);
    //        std::ofstream writer(buff);
    //        for (int j=0; j< this->rows; j++){

    //            for (int k=0; k< this->cols; k++){
    //                //writer<<_data[i][j][k]<< " " << _hits[i][j][k]<<" " << sqrt((_covariance[i][j][k]/_hits[i][j][k]) -_data[i][j][k])<<" ";
    //                writer<<j<<" "<<k<<" " <<_data[i][j][k]/_hits[i][j][k]<< std::endl;
    //            }

    //        }
    //        writer.close();
    //    }

    //    std::cout<<"done"<<std::endl;
    std::ifstream myfile ("layer_32_4096_ANN.txt");
    float m;
    cv::Mat ANN(540,960,CV_32FC1);
    cv::Point p;
    for (int i=0; i<540; i++){
        for (int j=0; j<960; j++){
            int buco;
            myfile>>buco;myfile>>buco;
            myfile>>m;
            p.x=j;
            p.y=i;
            ANN.at<float>(p)=(m-1)*3000+127;
        }
    }
    myfile.close();
    cv::flip(ANN,ANN,0);
    double min;
    double max;
    ANN.convertTo(ANN,CV_8UC1);
    cv::minMaxIdx(ANN,&min,&max);
    cv::Mat dest;
    for(int colormap =0;colormap<1;colormap++){
        cv::applyColorMap(ANN,dest,colormap);
        cv::imwrite("ANN_OUT.PGM",dest);
    }
}




void CalibrationMatrix::dumpCovariance(){
    for (int i=0; i<layers; i++){
        cv::Mat errorImage(rows,cols,CV_32FC1);
        cv::Mat error(rows,cols,CV_8UC1);
        errorImage=cv::Mat::zeros(rows,cols,CV_32FC1);
        cv::Point p;
        for (int j=0; j< rows; j++){

            for (int k=0; k< cols; k++){
                p.y=j;
                p.x=k;
                float v= sqrt(((_data[i][j][k]*_data[i][j][k])/_hits[i][j][k])-(_data[i][j][k]/_hits[i][j][k])*(_data[i][j][k]/_hits[i][j][k]));
                errorImage.at<float>(p)=v;//(v-1)*3000+127;

            }

        }
        cv::flip(errorImage,errorImage,0);
        double min;
        double max;

        //errorImage.convertTo(error,CV_8UC1, 255.0/(max - min), -min * 255.0/(max - min));
        cv::minMaxIdx(errorImage,&min,&max);
        std::cout << "m: "<<min << " M: "<<max<<std::endl;
        //        errorImage.convertTo(error,CV_8UC1,255/max);
        errorImage.convertTo(error,CV_8UC1, 255.0/(max - min), -min * 255.0/(max - min));
        //cv::convertScaleAbs(errorImage, error, 255 / max);
        cv::Mat dest;
        char filename[50];
        for(int colormap =0;colormap<1;colormap++){
            cv::applyColorMap(error,dest,colormap);
            sprintf(filename,"covariances_%d.pgm",i);
            cv::imwrite(filename,dest);


        }

    }
}







