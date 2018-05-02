#include "elephant_port.h"

elephant_port::elephant_port()
{
    len_left=0;
    len_right=0;
    right_top.x=0;
    left_top.y=999;
    left_top.y=999;
    right_top.y=999;

}


void elephant_port::Grow(int detaleft,int detaright)
{
    //初始化
    cv::Mat image,thresholded,fill;
    if(img.type()!=0)
        cv::cvtColor(img,image,CV_RGB2GRAY);
    else
        image=img;
    cv::threshold(image,thresholded,200,255,cv::THRESH_BINARY);
    thresholded=~thresholded;

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
        if(itc->size()<400)
            itc=contours.erase(itc);
        else{
            ++itc;
        }
    }
    fill=cv::Mat(image.size(),CV_8U,cv::Scalar(255));
    image=cv::Mat(image.size(),CV_8U,cv::Scalar(255));
    cv::drawContours(image,
                     contours,
                     -1,            //绘制所有轮廓
                     cv::Scalar(0), //颜色为黑
                     2);            //轮廓线宽度为2
    cv::drawContours(fill,
                     contours,
                     -1,            //绘制所有轮廓
                     cv::Scalar(0), //颜色为黑
                     CV_FILLED);            //轮廓线宽度为2


    int cols=image.cols,rows=image.rows;

    std::vector<int> colCount(cols,0),yMin(cols,100);

    for(int j=0;j<rows;j++){
        uchar *data=image.ptr<uchar>(j);
        for(int i=0;i<cols;i++){
            if(data[i]<10){
                colCount[i]++;
                if(yMin[i]>j)
                    yMin[i]=j;
                if(i+j<left_top.x+left_top.y)
                    left_top=cv::Point2f(i,j);
                if(cols-i+j<cols-right_top.x+right_top.y)
                    right_top=cv::Point2f(i,j);
            }
        }
    }
    bool flag=true;
    while(flag){
        flag=false;
        for(int i=1;i<7;i++)
            if(left_top.x+i>=0&&left_top.x+i<cols&&colCount[left_top.x]>colCount[left_top.x+i])
                flag=true;
        qDebug()<<"left - colCount["<<left_top.x<<"]:"<<colCount[left_top.x];
        if(flag)left_top.x++;
    }
    left_top.x+=detaleft;
    flag=true;
    while(flag){
        flag=false;
        for(int i=1;i<7;i++)
            if(right_top.x-i<cols&&right_top.x-i>=0&&colCount[right_top.x]>colCount[right_top.x-i])
                flag=true;
        qDebug()<<"right - colCount["<<right_top.x<<"]:"<<colCount[right_top.x];
        if(flag)right_top.x--;
    }
    right_top.x-=detaright;
    qDebug()<<"求长度";
    left_top.y=yMin[left_top.x];
    right_top.y=yMin[right_top.x];
    flag=true;
    while(flag){
        qDebug()<<"len_left:"<<len_left<<"left_topy"<<left_top.y<<"left top x"<<left_top.x;
        flag=false;
        for(int i=1;i<14;i++)
            if((left_top.y+len_left+i)<rows&&fill.at<uchar>(left_top.y+len_left+i,left_top.x)<20)
                flag=true;
        if(flag)len_left++;
    }
    qDebug()<<"len_left:"<<len_left;

    flag=true;
    while(flag){
        flag=false;
        for(int i=1;i<14;i++)
            if((right_top.y+len_right+i)<rows&&fill.at<uchar>(right_top.y+len_right+i,right_top.x)<20)
                flag=true;
        if(flag)len_right++;
    }
    qDebug()<<"right_left:"<<len_right;
    //划线

    mask=~fill;
    for(int j=0;j<rows;j++){
        uchar *data=mask.ptr<uchar>(j);
        for(int i=0;i<cols;i++){
            if((left_top.x<(cols/4)&&i<left_top.x&&j-left_top.y<len_left))
                data[i]=0;
            if(right_top.x>(3*cols)/4&&i>right_top.x&&j-right_top.y<len_right)
                data[i]=0;
        }
    }
    cv::imshow("mask",mask);
}

int elephant_port::d(cv::Vec3b a, cv::Vec3b b)
{
    return std::sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]));
}

std::vector<int> sort(std::vector<std::pair<double,double> > &len)
{
    double min=9999999;
    std::vector<int> number(len.size()),answer;
    for(int i=0;i<number.size();i++)
        number[i]=i;
    solu(len,number,1,0,min,answer);
    return answer;
}

void solu(std::vector<std::pair<double,double> > &data, std::vector<int> &number, int left, double sum, double &min, std::vector<int> &answer)
{
    if(left>=data.size()){
        if(sum<min){
            min=sum;
            answer=number;
        }
        return;
    }
    for(int i=left;i<number.size();i++){
        std::swap(number[left],number[i]);
        double count=(data[number[left-1]].second-data[number[left]].first)*(data[number[left-1]].second-data[number[left]].first);
        solu(data,number,left+1,sum+count,min,answer);
        std::swap(number[left],number[i]);
    }
}
