#include "histogram.h"

cv::MatND Histogram1D::getHistogram(const cv::Mat &image)
{
    cv::MatND hist;
    cv::calcHist(&image,
                 1,
                 channels,
                 cv::Mat(),
                 hist,
                 1,
                 histSize,
                 ranges
                 );
    return hist;
}

cv::Mat Histogram1D::getHistogramImage(const cv::Mat &image)
{
    //计算直方图
    cv::MatND hist=getHistogram(image);
    //获取最大最小值
    double maxVal=0,minVal=0;
    cv::minMaxLoc(hist,&minVal,&maxVal,0,0);
    //直方图图像
    cv::Mat histImg(histSize[0],histSize[0],CV_8U,cv::Scalar(255));
    //最高点的百分之九十
    int hpt=static_cast<int>(0.9*histSize[0]);
    //绘制线段
    for(int h=0;h<histSize[0];h++){
        float binVal=hist.at<float>(h);
        int intensity=static_cast<int>(binVal*hpt/maxVal);
        //绘制两点之间的线
        cv::line(histImg,
                 cv::Point(h,histSize[0]),
                cv::Point(h,histSize[0]-intensity),
                cv::Scalar::all(0));
    }
    return histImg;
}

cv::Mat Histogram1D::applyLookUp(const cv::Mat &image, const cv::Mat &lookup)
{
    cv::Mat result;
    cv::LUT(image,lookup,result);
    return result;
}

cv::MatND ColorHistogram::getHistogram(const cv::Mat &image)
{
    cv::MatND hist;
    cv::calcHist(&image,
                 1,
                 channels,
                 cv::Mat(),
                 hist,
                 3,
                 histSize,
                 ranges
                 );
    return hist;
}

