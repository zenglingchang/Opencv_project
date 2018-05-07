#ifndef DELAUNAY_H
#define DELAUNAY_H
#include"help.h"

class Delaunay
{
    cv::Subdiv2D subdiv;
    cv::Rect rect;
    map<pair<float,float>,int> Map;
public:
    Delaunay(cv::Rect rect);
    void insert(std::vector<cv::Point2f> &Points);
    void drawDelaunay(cv::Mat &src,cv::Mat &dst,Scalar);
};

#endif // DELAUNAY_H
