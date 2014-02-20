#include "fancyWindow.h"
#include <iostream>
#include <fstream>
#include <QTime>


FancyWindow::FancyWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::FancyWindow)
{
    ui->setupUi(this);
    this->viewer = this->ui->myFancyViewer;

}


FancyWindow::~FancyWindow()
{
    delete ui;
}

void FancyWindow::on_toggleError_clicked(bool checked)
{
    if(sub->applyCorrection) sub->applyCorrection=false;
    else sub->applyCorrection=true;
    std::cout << "apply correction "<< sub->applyCorrection<<std::endl;
    this->ui->correcting->setChecked(sub->applyCorrection);
}


void FancyWindow::on_recordingData_clicked()
{
    if(sub->recordData) sub->recordData=false;
    else sub->recordData=true;
    std::cout << "recording data "<< sub->recordData<<std::endl;
    this->ui->recording->setChecked(sub->recordData);
}

void FancyWindow::on_rejecter_clicked()
{
    if(this->viewer->rejectPoints) this->viewer->rejectPoints=false;
    else this->viewer->rejectPoints=true;
    std::cout << "rejecting data "<< this->viewer->rejectPoints<<std::endl;
}

void FancyWindow::on_saveData_clicked()
{
    std::cout << "serialization called"<<std::endl;
    this->sub->multiplier.serialize("prova.txt");
}

void FancyWindow::on_loadCalibData_clicked()
{
    std::cout << "deserialization called"<<std::endl;
    this->sub->multiplier.deserialize("prova.txt");
}

void FancyWindow::on_saveSensorImage_clicked()
{
    std::cout << "dumping images"<<std::endl;
    this->sub->multiplier.dumpSensorImages();
    //this->sub->multiplier.dumpCovariance();
}

void FancyWindow::on_voxelLeaf_valueChanged(int arg1)
{
    this->sub->voxelLeaf=(float)arg1;
}

void FancyWindow::on_normalRejection_valueChanged(double arg1)
{
    this->sub->normalRejection=(float)arg1;
}

void FancyWindow::on_planeModelInliers_clicked()
{
    if(this->sub->planeModelInliers) this->sub->planeModelInliers=false;
    else this->sub->planeModelInliers=true;
}

void FancyWindow::on_pointSize_valueChanged(int arg1)
{
    this->viewer->pointSize=arg1;
}

void FancyWindow::on_r_valueChanged(int arg1)
{
    this->viewer->br=arg1;
    this->viewer->changeBgColor();
}

void FancyWindow::on_g_valueChanged(int arg1)
{
    this->viewer->bg=arg1;
    this->viewer->changeBgColor();
}

void FancyWindow::on_b_valueChanged(int arg1)
{
    this->viewer->bb=arg1;
    this->viewer->changeBgColor();
}

void FancyWindow::on_referenceMM_valueChanged(int arg1)
{
    this->sub->refenceDistance=arg1;
}

void FancyWindow::on_referenceError_clicked()
{
    if(this->sub->computeRefenceDistance) this->sub->computeRefenceDistance=false;
    else this->sub->computeRefenceDistance=true;
}
