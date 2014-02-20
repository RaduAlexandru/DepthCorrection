#ifndef FANCYVIEWER_H
#define FANCYVIEWER_H

#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <QDialog>
#include <QThread>
#include <QLabel>
#include <QTime>
#include <QImage>
#include <unistd.h>
#include <iostream>
#include <QLineEdit>
#include <stdio.h>
#include <QGLViewer/qglviewer.h>
#include "FancyQueue.h"
#include "shared/CalibrationMatrix.h"

class FancyViewer : public QGLViewer
{
public:
    FancyViewer(QWidget *parent);
    FancyQueue* queue;
    QueuePayload data;
    bool rejectPoints;
    int pointSize;
    int br,bg,bb;
    void changeBgColor();
private :
    void drawPointcloud();

    virtual void draw();
    void drawCentralNormal(Eigen::Vector4f p, Eigen::Vector4f c);
    void drawNormals();
    virtual void init();
    virtual void animate();
};

#endif
