#ifndef DETECTIONTOOL_H
#define DETECTIONTOOL_H

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "objectdetection.h"
#include "cameracontroller.h"
#include "const.h"

#include <QApplication>
#include <QtGui>
#include <QObject>

class DetectionTool : public QObject
{
    Q_OBJECT
    
public:
    DetectionTool(QApplication *app);
    ~DetectionTool();
    int go();
    
public slots:
    void testDetectionImg();
    void testDetectionImg_process();
    void toBegin();
    
private:
    QWidget *fen;
    QVBoxLayout *vbox_begin;
    QVBoxLayout *vbox_testimg;
    QPushButton *bt1;
    QApplication *app;
    QLabel *lbl1;
    QLabel *lbl2;
    QFormLayout *fl;
    QLineEdit *file;
    QLineEdit *obj;
    
    
    void center(QWidget &widget ,int WIDTH , int HEIGHT);
    void empty();
    QImage Mat2QImage(cv::Mat const& src);
};

#endif // DETECTIONTOOL_H
