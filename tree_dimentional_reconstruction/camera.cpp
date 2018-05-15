#include "camera.h"

Camera::Camera()
{

}

void Camera::calibrate(std::vector<string> filelist)
{
    //初始化
    int len=29;                             //棋盘每格为29mmX29mm
    cv::Mat image;                          //存储图像
    std::vector<cv::Point2f> imageCorners;  //暂存二维角点
    std::vector<cv::Point3f> objectCorners; //暂存三维角点
    cv::Size boardSize(9,6);                //9*6的棋盘
    std::vector<std::vector<cv::Point2f>> imagePoints;//储存储存二维角点数组
    std::vector<std::vector<cv::Point3f>> objectPoints;//储存储存三维角点数组

    //棋盘三维坐标
    for(int i=0;i<boardSize.height;i++)
        for(int j=0;j<boardSize.width;j++)
            objectCorners.push_back(cv::Point3f((i+1)*len,(j+1)*len,0.0f));

    for(int i=0;i<filelist.size();i++){
        image=cv::imread(filelist[i],0);
        double ratio=std::sqrt((image.cols*image.rows)/500000);//棋盘缩小
        qDebug()<<filelist[i].data();
        qDebug()<<ratio<<"  "<<image.cols<<"  "<<image.rows;
        cv::resize(image,image,cv::Size(image.cols/ratio,image.rows/ratio),0,0,INTER_AREA);
        //棋盘格角点检测
        qDebug()<<i;
        bool found=cv::findChessboardCorners(image,boardSize,imageCorners,CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        //若发现角点再进行亚精度像素角点检测
        if(found){
        cv::cornerSubPix(
                    image,
                    imageCorners,
                    cv::Size(5,5),          //搜索边长大小的一半
                    cv::Size(-1,-1),
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 30, 0.1));
            imagePoints.push_back(imageCorners);
            objectPoints.push_back(objectCorners);
            qDebug()<<i;
        }
    }

    //获取相机参数和旋转平移向量
    std::vector<cv::Mat> rotation_vectors,translation_vector;
    //计算内参矩阵和畸变矩阵
    cv::calibrateCamera(objectPoints,imagePoints,image.size(),in_matrix,distortion_coeffs,rotation_vectors,translation_vector);
    //计算误差
    std::vector<cv::Point2f> image_point2;
    double total_error=0.0;
    for(int i=0;i<imagePoints.size();i++){
        std::vector<cv::Point3f> tempPoints=objectPoints[i];
        //计算得到新投影点储存在image_point2中
        projectPoints(tempPoints,rotation_vectors[i],translation_vector[i],in_matrix,distortion_coeffs,image_point2);
        vector<Point2f> tempImagePoint = imagePoints[i];
        //计算误差
        Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);
        Mat image_points2Mat = Mat(1,image_point2.size(), CV_32FC2);
        for (int j = 0 ; j < tempImagePoint.size(); j++){
            image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_point2[j].x, image_point2[j].y);
            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }

        double err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        total_error+=err;
        qDebug()<<"第"<<i+1<<"幅图片平均误差为："<<err<<"Px";
    }
    qDebug()<<"总平均误差为："<<total_error/imagePoints.size()<<"Px";
    qDebug()<<"内参矩阵: ";
    for(int i=0;i<3;i++)
        qDebug()<<in_matrix.at<double>(i,0)<<"\t"<<in_matrix.at<double>(i,1)<<"\t"<<in_matrix.at<double>(i,2)<<"  ";
    focal_length=0.5*(in_matrix.at<double>(0) + in_matrix.at<double>(4));
    principle_point=Point2d(in_matrix.at<double>(2), in_matrix.at<double>(5));
}
