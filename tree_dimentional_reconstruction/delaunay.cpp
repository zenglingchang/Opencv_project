#include "delaunay.h"

Delaunay::Delaunay(Rect rect)
{
    ratio=0;
    this->rect=rect;
    subdiv=cv::Subdiv2D(rect);
}

void Delaunay::insert(std::vector<Point2f> &Points,std::vector<Point3f> &ObPoints)
{
    ObjectPoints=ObPoints;
    for(int i=0;i<Points.size();i++){
        subdiv.insert(Points[i]);
        Map[pair<float,float>(Points[i].x,Points[i].y)]=i;
    }
}
void Delaunay::GetFace(Mat &src, Mat &dst, std::vector<int> &FaceList)
{
    dst=src;
    std::vector<Vec6f> TriangleLists;
    subdiv.getTriangleList(TriangleLists);
    std::vector<cv::Point2f> pt(3);
    int num[3];
    double Size=TriangleLists.size()*3;
//    qDebug()<<"Size: "<<Size;
    for(size_t i=0;i<TriangleLists.size();i++){
        Vec6f t=TriangleLists[i];
        pt[0]=cv::Point2f(t[0],t[1]);
        pt[1]=cv::Point2f(t[2],t[3]);
        pt[2]=cv::Point2f(t[4],t[5]);
        if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
        {
            for(int j=0;j<3;j++)
                num[j]=Map[pair<float,float>(pt[j].x,pt[j].y)];
            for(int j=0;j<3;j++){
                ratio+=d(ObjectPoints[num[j]],ObjectPoints[num[(j+1)%3]])/d(pt[j],pt[(j+1)%3])/Size;
                FaceList.push_back(Map[pair<float,float>(pt[j].x,pt[j].y)]);
            }
            line(dst, pt[0], pt[1], cv::Scalar(255,255,255), 1, CV_AA, 0);
            line(dst, pt[1], pt[2], cv::Scalar(255,255,255), 1, CV_AA, 0);
            line(dst, pt[2], pt[0], cv::Scalar(255,255,255), 1, CV_AA, 0);
        }
    }
//    qDebug()<<"Ratio: "<<ratio;
}
void Delaunay::drawDelaunay(Mat &src, Mat &dst, Scalar delaunay_color)
{
    dst=src;
    std::vector<Vec6f> TriangleLists;
    subdiv.getTriangleList(TriangleLists);
    std::vector<cv::Point2f> pt(3);
    for(size_t i=0;i<TriangleLists.size();i++){
        Vec6f t=TriangleLists[i];
        pt[0]=cv::Point2f(t[0],t[1]);
        pt[1]=cv::Point2f(t[2],t[3]);
        pt[2]=cv::Point2f(t[4],t[5]);
        if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
        {
            line(dst, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
            line(dst, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
            line(dst, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
        }
    }
}
float d(Point2f &p1, Point2f &p2)
{
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

float d(Point3f &p1, Point3f &p2)
{
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
}
