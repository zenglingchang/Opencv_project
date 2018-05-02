#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    cv::Mat image=cv::imread("C:\\Users\\77470\\Desktop\\computer_vision\\animals.jpg"),result,thresholded;
    //cv::imshow("test",image);

//    //直方图
//    Histogram1D histogram;
//    cv::equalizeHist(image,result);
//    cv::Mat hist=histogram.getHistogramImage(result);
//    cv::imshow("hist",hist);

    //阈值处理
    cv::cvtColor(image,thresholded,CV_RGB2GRAY);
    cv::threshold(thresholded,thresholded,60,255,cv::THRESH_BINARY);
    //cv::imshow("thresholded",thresholded);
    //像素取反
    thresholded=~thresholded;


    //先闭再开
//    cv::dilate(thresholded,result,cv::Mat());
//    cv::erode(result,result,cv::Mat());
//    cv::erode(result,result,cv::Mat());
//    cv::dilate(result,result,cv::Mat());

//5X5的结构
    cv::Mat element5(5,5,CV_8U,cv::Scalar(1));
    cv::morphologyEx(thresholded,result,cv::MORPH_CLOSE,element5);
    cv::morphologyEx(result,result,cv::MORPH_OPEN,element5);
    //cv::imshow("close_image",result);

    std::vector<std::vector<cv::Point>> contours;//轮廓向量
    cv::findContours(result,
                     contours,//轮廓数组
                     CV_RETR_EXTERNAL,//获取外轮廓
                     CV_CHAIN_APPROX_NONE//获取每个轮廓的每个像素
                     );
    //过滤极大极小值
    int cmin=50,cmax=1000;
    auto itc = contours.begin();
    while(itc!=contours.end()){
        if(itc->size()<cmin||itc->size()>cmax)
            itc=contours.erase(itc);
        else
            ++itc;
    }
    std::vector<std::vector<cv::Point> > contours_poly(contours.size());
    std::vector<cv::Rect> BoundRect(contours.size());
    //获取围绕轮廓的最小方框
    for(int i=0;i<contours.size();i++){
        cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
        BoundRect[i]=cv::boundingRect(cv::Mat(contours_poly[i]));
        cv::rectangle(image,BoundRect[i],cv::Scalar::all(255));
    }
    cv::Mat white_image(image.size(),CV_8U,cv::Scalar(255));
    cv::drawContours(white_image,
                     contours_poly,
                     -1,            //绘制所有轮廓
                     cv::Scalar(0), //颜色为黑
                     2);            //轮廓线宽度为2
    ui->number->setText(QString::number(contours.size(),10));
    cv::imshow("contours",white_image);
    cv::imshow("image",image);
}
