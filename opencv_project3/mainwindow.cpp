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
    cv::Mat src=cv::imread("C:\\Users\\77470\\Desktop\\computer_vision\\elephant.jpg"),grabimage,result,thresholded;

    //grabcut算法 分割前景图
    cv::Rect rectangle(60,178,850,444);
    GrabCut grabcut;
    grabimage=grabcut.solution(src,rectangle);\
    cv::imshow("grabcut",grabimage);

//    Histogram1D histogram;
    //依据直方图发现在70左右出现峰值
    cv::cvtColor(grabimage,result,CV_RGB2GRAY);
    cv::equalizeHist(result,result);
    //cv::Mat hist=histogram.getHistogramImage(result);
    //cv::imshow("hist",hist);

    //二值化，并取补
    cv::threshold(result,thresholded,70,255,cv::THRESH_BINARY);
    thresholded=~thresholded;
    cv::imshow("thre",thresholded);

    //求轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholded,
                     contours,//轮廓数组
                     CV_RETR_EXTERNAL,//获取外轮廓
                     CV_CHAIN_APPROX_NONE//获取每个轮廓的每个像素
                     );
    //去除噪声轮廓
    auto itc = contours.begin();
    while(itc!=contours.end()){
        qDebug()<<itc->size();
        if(itc->size()<400)
            itc=contours.erase(itc);
        else{
            qDebug()<<itc->size();
            ++itc;
        }
    }

    cv::Mat white_image(src.size(),CV_8U,cv::Scalar(255));
    std::vector<std::vector<cv::Point> > contours_poly(contours.size());
    std::vector<cv::Rect> BoundRect(contours.size());
    std::vector<elephant_port> buffer(contours.size());

    //获取围绕轮廓的最小方框，并依据方框分割图像

    for(int i=0;i<contours.size();i++){
        cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
        BoundRect[i]=cv::boundingRect(cv::Mat(contours_poly[i]));
    }
    cv::drawContours(white_image,
                     contours,
                     -1,            //绘制所有轮廓
                     cv::Scalar(0), //颜色为黑
                     2);            //轮廓线宽度为2
    for(int i=0;i<contours.size();i++){
        buffer[i].img=grabimage(BoundRect[i]);
        buffer[i].center=cv::Point(BoundRect[i].width/2,BoundRect[i].height/2);
        cv::imwrite("D:\\qt_project\\opencv_project3\\project3\\"+QString::number(i).toStdString()+".jpg",buffer[i].img);
    }
    cv::imshow("contours",white_image);
    cv::imshow("src",src);


}

void MainWindow::on_pushButton_2_clicked()
{
    std::vector<elephant_port> buffer(8);
    for(int i=0;i<8;i++){
        buffer[i].img=cv::imread("D:\\qt_project\\opencv_project3\\project3\\"+QString::number(i).toStdString()+".jpg");
        buffer[i].Grow();
    }
    buffer[2].Grow(0,5);
    buffer[3].Grow(4);
    buffer[4].Grow(10);
    buffer[5].Grow(0,5);
//    buffer[6].Grow(2);
    std::vector<std::pair<double,double>> len(8);
    for(int i=0;i<8;i++){
        len[i].first=buffer[i].len_left,len[i].second=buffer[i].len_right;
        cv::imshow(QString::number(i).toStdString(),buffer[i].img);
    }
    std::vector<int> number=sort(len);
    for(auto a:number)
        qDebug()<<a;
    qDebug()<<"开始匹配";
    cv::Mat answer(500,900,CV_8UC3,cv::Scalar(255,255,255)),imageROI;
    int x=100,y=50;
    double ratio=1;
    for(int j=0,i=0;j<8;j++){
        i=number[j];
        cv::resize(buffer[i].img,buffer[i].img,cv::Size(buffer[i].img.cols*ratio,buffer[i].img.rows*ratio),0,0,cv::INTER_AREA);
        cv::resize(buffer[i].mask,buffer[i].mask,cv::Size(buffer[i].mask.cols*ratio,buffer[i].mask.rows*ratio),0,0,cv::INTER_AREA);
        qDebug()<<"img_size: cols"<<buffer[i].img.cols<<"  rows:"<<buffer[i].img.rows;
        qDebug()<<"mask_size: cols"<<buffer[i].mask.cols<<"  rows:"<<buffer[i].mask.rows;
        imageROI=answer(cv::Rect(x,y,buffer[i].img.cols,buffer[i].img.rows));
        buffer[i].img.copyTo(imageROI,buffer[i].mask);
        cv::Point temp(x+buffer[i].left_top.x*ratio,y+buffer[i].left_top.y*ratio);
        for(int len=buffer[i].len_left*ratio;len>=0;len--){
            answer.at<cv::Vec3b>(temp.y+len,temp.x-2)=answer.at<cv::Vec3b>(temp.y+len,temp.x-3)/2+answer.at<cv::Vec3b>(temp.y+len,temp.x-2)/2;
            answer.at<cv::Vec3b>(temp.y+len,temp.x+2)=answer.at<cv::Vec3b>(temp.y+len,temp.x+3)/2+answer.at<cv::Vec3b>(temp.y+len,temp.x+2)/2;
            answer.at<cv::Vec3b>(temp.y+len,temp.x-1)=3*(answer.at<cv::Vec3b>(temp.y+len,temp.x-2)/4)+answer.at<cv::Vec3b>(temp.y+len,temp.x+2)/4;
            answer.at<cv::Vec3b>(temp.y+len,temp.x+1)=3*(answer.at<cv::Vec3b>(temp.y+len,temp.x+2)/4)+answer.at<cv::Vec3b>(temp.y+len,temp.x-2)/4;
            answer.at<cv::Vec3b>(temp.y+len,temp.x)=answer.at<cv::Vec3b>(temp.y+len,temp.x-1)/2+answer.at<cv::Vec3b>(temp.y+len,temp.x+1)/2;
        }
        qDebug()<<"number["<<j<<"]:"<<i;
        if(j==7)break;
        x+=buffer[i].right_top.x*ratio,y+=buffer[i].right_top.y*ratio;
        ratio*=buffer[i].len_right/buffer[number[j+1]].len_left;
        x -= buffer[number[j+1]].left_top.x*ratio,y -= buffer[number[j+1]].left_top.y*ratio;

    }
//    cv::medianBlur(answer,answer,3);
    cv::imshow("answer",answer);

}
