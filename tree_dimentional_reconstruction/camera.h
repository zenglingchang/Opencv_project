#ifndef CAMERA_H
#define CAMERA_H
#include"help.h"

class Camera
{
public:
    Camera();
    cv::Mat in_matrix;          //内参矩阵
    cv::Mat distortion_coeffs;  //畸变参数
};

#endif // CAMERA_H
