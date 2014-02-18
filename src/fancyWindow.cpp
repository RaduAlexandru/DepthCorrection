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
}
