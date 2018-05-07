#ifndef FEATURE_DEAL_H
#define FEATURE_DEAL_H
#include"help.h"


class Feature_Deal
{
public:
    Feature_Deal(cv::Mat &ima);
    cv::Mat descriptors;                    //特征描述子
    std::vector<cv::KeyPoint> keypoints;    //关键点数组
    double ratio;                              //缩放倍率
    cv::Ptr<cv::Feature2D> Detector=cv::xfeatures2d::SIFT::create();//SURF 特征检测器
    //cv::xfeatures2d::SIFT::create(); SIFT特诊检测器
    cv::Mat image;
};
void match_feature(Feature_Deal &a,Feature_Deal &b,vector<Point2f>& p1, vector<Point2f>& p2,cv::Rect );
#endif // FEATURE_DEAL_H
