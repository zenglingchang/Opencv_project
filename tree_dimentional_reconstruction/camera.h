#ifndef CAMERA_H
#define CAMERA_H
#include"help.h"

class Camera
{
public:
    Camera();
    cv::Mat in_matrix;          //内参矩阵
    cv::Mat distortion_coeffs;  //畸变参数
    double focal_length;        //焦距
    Point2d principle_point;    //光心点坐标
    void calibrate(std::vector<std::string> filelist); //根据棋盘格计算内参
};

#endif // CAMERA_H
