#ifndef GRABCUT_H
#define GRABCUT_H
#include"help.h"


class GrabCut
{
    cv::Mat result;             //分割(四种可能的值)
    cv::Mat bgModel,fgModel;
public:
    GrabCut();
    cv::Mat solution(cv::Mat image,cv::Rect rectangle);
};

#endif // GRABCUT_H
