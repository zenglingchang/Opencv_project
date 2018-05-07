#include "delaunay.h"

Delaunay::Delaunay(Rect rect)
{
    this->rect=rect;
    subdiv=cv::Subdiv2D(rect);
}

void Delaunay::insert(std::vector<Point2f> &Points)
{
    for(int i=0;i<Points.size();i++){
        subdiv.insert(Points[i]);
        Map[pair<float,float>(Points[i].x,Points[i].y)]=i;
    }
}

void Delaunay::drawDelaunay(Mat &src, Mat &dst, Scalar delaunay_color)
{
    dst=src;
    std::vector<Vec6f> TriangleLists;
    subdiv.getTriangleList(TriangleLists);
    std::vector<cv::Point2f> pt(3);
    qDebug()<<"Triangle size: "<<TriangleLists.size();
    fstream file("D:\\qt_project\\tree_dimensional_reconstruction\\tree_dimentional_reconstruction\\model.txt",ios::app);
    for(size_t i=0;i<TriangleLists.size();i++){
        Vec6f t=TriangleLists[i];
        pt[0]=cv::Point2f(t[0],t[1]);
        pt[1]=cv::Point2f(t[2],t[3]);
        pt[2]=cv::Point2f(t[4],t[5]);
        if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
        {
            qDebug()<<Map[pair<float,float>(pt[0].x,pt[0].y)]<<","<<Map[pair<float,float>(pt[1].x,pt[1].y)]
                    <<","<<Map[pair<float,float>(pt[2].x,pt[2].y)]<<",";
            file<<"f "<<Map[pair<float,float>(pt[0].x,pt[0].y)]<<" "<<Map[pair<float,float>(pt[1].x,pt[1].y)]
                    <<" "<<Map[pair<float,float>(pt[2].x,pt[2].y)]<<endl;
            line(dst, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
            line(dst, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
            line(dst, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
        }
    }
    file.close();
}