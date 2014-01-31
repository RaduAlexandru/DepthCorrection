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

private:
    Ui::FancyWindow *ui;



};

#endif
