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
}


void FancyWindow::on_recordingData_clicked()
{
    if(sub->recordData) sub->recordData=false;
    else sub->recordData=true;
    std::cout << "recording data "<< sub->recordData<<std::endl;
}
