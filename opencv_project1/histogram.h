#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include"help.h"

class Histogram1D
{
private:
    int histSize[1];
    float hranges[2];//像素的最小及最大值
    const float *ranges[1];
    int channels[1];//仅用到一个通道
public:
    Histogram1D(){
        histSize[0]=256;
        hranges[0]=0.0;
        hranges[1]=255.0;
        ranges[0]=hranges;
        channels[0]=0;
    }
    cv::MatND getHistogram(const cv::Mat &image);
    cv::Mat getHistogramImage(const cv::Mat &image);
    cv::Mat applyLookUp(const cv::Mat &image,const cv::Mat &lookup);
};

class ColorHistogram{
private:
    int histSize[3];
    float hranges[2];
    const float *ranges[3];
    int channels[3];
public:
    ColorHistogram(){
        histSize[0]=histSize[1]=histSize[2]=256;
        hranges[0]=0.0;
        hranges[1]=255.0;
        ranges[0]=hranges;
        ranges[1]=hranges;
        ranges[2]=hranges;
        channels[0]=0;
        channels[1]=1;
        channels[2]=2;
    }
    cv::MatND getHistogram(const cv::Mat &image);
};

#endif // HISTOGRAM_H
