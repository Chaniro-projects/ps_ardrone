#include "detectiontool.h"

DetectionTool::DetectionTool(QApplication *app) : vbox(0)
{
    this->app = app;
    fen = new QWidget();
    toBegin();
}

DetectionTool::~DetectionTool() {
    delete vbox;
}

int DetectionTool::go() {
    fen->show();
    return app->exec();
}

void DetectionTool::testDetectionImg()
{
    empty();
    
    file = new QLineEdit();
    obj = new QLineEdit();
    
    fl = new QFormLayout();
    fl->addRow("Image file", file);
    fl->addRow("Object", obj);
    
    bt1 = new QPushButton("Process");
    QObject::connect(bt1, SIGNAL(clicked()), this, SLOT(testDetectionImg_process()));
    
    vbox->addLayout(fl);
    vbox->addWidget(bt1);
}

void DetectionTool::testDetectionImg_process()
{
    empty();
    
    std::string imgPath = PATH_PKG"object/";
    imgPath += file->text().toUtf8().constData();
    QFile f(imgPath.c_str());
    
    if(f.exists() && ObjectDetection::getInstance().exist(obj->text().toUtf8().constData())) {
        cv::Mat img = cv::imread(imgPath);
        
        ObjectDetection::DetectionResult res = ObjectDetection::getInstance().detectObject(img, obj->text().toUtf8().constData(), false, 50, 300);
        
        if(res.detected) {
            QString str = "Taille: " + QString::number(res.size);
            lbl1 = new QLabel(str);
            lbl2 = new QLabel();
            
            lbl2->setPixmap(QPixmap::fromImage(Mat2QImage(*res.imgRange)));
            
            fen->setFixedSize(800, 600);
            center(*fen, 800, 600);
            
            vbox->addWidget(lbl2);
            vbox->addWidget(lbl1);
        }
        else
            toBegin();
    }
    else
        toBegin();
}

void DetectionTool::toBegin()
{
    fen->setFixedSize(300, 300);
    fen->setWindowTitle("Object detection tools");
    center(*fen, 300, 300);
    
    vbox = new QVBoxLayout();
    
    bt1 = new QPushButton("Detection test from image");
    QObject::connect(bt1, SIGNAL(clicked()), this, SLOT(testDetectionImg()));
    vbox->addWidget(bt1);
    
    fen->setLayout(vbox);
}

void DetectionTool::center(QWidget &widget ,int WIDTH , int HEIGHT)
{
  int x, y;
  int screenWidth;
  int screenHeight;
 
  QDesktopWidget *desktop = QApplication::desktop();
 
  screenWidth = desktop->width();
  screenHeight = desktop->height();
 
  x = (screenWidth - WIDTH) / 2;
  y = (screenHeight - HEIGHT) / 2;
 
  widget.setGeometry(x, y, WIDTH, HEIGHT);
 
}

void DetectionTool::empty() {
    delete fen;
    fen = new QWidget();
}

QImage DetectionTool::Mat2QImage(cv::Mat const& src)
{
     cv::Mat temp;
     cvtColor(src, temp,CV_BGR2RGB);
     QImage dest((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
     QImage dest2(dest);
     dest2.detach(); // enforce deep copy
     return dest2;
}
