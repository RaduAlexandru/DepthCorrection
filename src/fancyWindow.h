#ifndef FANCYWINDOW_H
#define FANCYWINDOW_H
#include "mySubscriber.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <QDialog>
#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QTime>
#include <QImage>
#include "ui_fancyWindow.h" //remember me
#include <unistd.h>
#include <iostream>
#include <QLineEdit>
#include <stdio.h>
#include <QStatusBar>
#include "fancyViewer.h"
#include "FancyQueue.h"
#include <QString>
#include <string>


namespace Ui {
    class FancyWindow;
}


class FancyWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit FancyWindow(QWidget *parent = 0);
    ~FancyWindow();
    void subscriber(MySubscriber* m);
    FancyViewer* viewer;
    MySubscriber* sub;
private slots:
    void on_toggleError_clicked(bool checked);
    void on_recordingData_clicked();

    void on_rejecter_clicked();

    void on_saveData_clicked();

    void on_loadCalibData_clicked();

    void on_saveSensorImage_clicked();

    void on_voxelLeaf_valueChanged(int arg1);

    void on_normalRejection_valueChanged(double arg1);

    void on_planeModelInliers_clicked();

    void on_pointSize_valueChanged(int arg1);

    void on_r_valueChanged(int arg1);

    void on_g_valueChanged(int arg1);

    void on_b_valueChanged(int arg1);

    void on_referenceMM_valueChanged(int arg1);

    void on_referenceError_clicked();

private:
    Ui::FancyWindow *ui;



};

#endif
