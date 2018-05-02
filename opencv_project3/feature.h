#ifndef FEATURE_H
#define FEATURE_H
#include"help.h"

class feature
{
public:
    feature(cv::Mat);
    cv::Mat descriptors;                    //特征描述子
    std::vector<cv::KeyPoint> keypoints;    //关键点数组
    cv::Ptr<cv::Feature2D> Detector=cv::xfeatures2d::SIFT::create();//SURF 特征检测器
    //cv::xfeatures2d::SIFT::create(); SIFT特诊检测器
    cv::Mat image;
};
void match_feature(feature &a, feature &b);
#endif // FEATURE_H
