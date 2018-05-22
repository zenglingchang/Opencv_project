#ifndef DELAUNAY_H
#define DELAUNAY_H
#include"help.h"

class Delaunay
{
    double ratio;
    cv::Subdiv2D subdiv;
    cv::Rect rect;
    map<pair<float,float>,int> Map;
    std::vector<Point3f> ObjectPoints;
public:
    Delaunay(cv::Rect rect);
    void insert(std::vector<cv::Point2f> &Points,std::vector<cv::Point3f> &ObPoints);
    void drawDelaunay(cv::Mat &src,cv::Mat &dst,Scalar);
    void GetFace(cv::Mat &src,cv::Mat &dst,std::vector<int> &FaceList);
    double GetRatio(){return ratio;}
};
float d(Point2f &p1,Point2f &p2);
float d(Point3f &p1,Point3f &p2);
#endif // DELAUNAY_H
