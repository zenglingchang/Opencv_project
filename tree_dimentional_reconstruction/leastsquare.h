#ifndef LEASTSQUARE_H
#define LEASTSQUARE_H
//最小二乘法优化
#include"help.h"
class LeastSquare
{
    cv::Mat r;              //旋转向量
    cv::Mat T;              //平移向量
    cv::Mat K;              //内参矩阵
    bool GradValid;         //梯度有效标志位
    double Cost;            //残差和
    double Grad[6];         //梯度
    double GradRatio[6];    //步长
    double dx;                 //计算梯度用dx=0.00001
public:
    LeastSquare();
    void init(Mat &r, Mat &T,Mat &K);
    void SetRT(cv::Mat &r,cv::Mat &T,cv::Mat &K){
        this->r=r;
        this->T=T;
        this->K=K;
    }
    double cost(cv::Mat &Proj,vector<Point2f> &ImagePoints,vector<Point3f> &ObjectPoints);                                                //损失函数
    void solution(vector<Point2f> &ImagePoints,vector<Point3f> &ObjectPoints);
    double Switch(cv::Mat &tempr,cv::Mat &tempT,double dx,int i);

    void GradClear(){                       //梯度无效重新计算
        for(int i=0;i<6;i++)
            Grad[i]=0;
    }

    double GetGrad(int &i,vector<Point2f> &ImagePoints,vector<Point3f> &ObjectPoints);                 //返回梯度

    void SetGrad(int &i,double &grad){      //设置梯度
        Grad[i]=grad;
    }
    bool operator()(Mat &r, Mat &T,Mat &K,vector<Point2f> &ImagePoints, vector<Point3f> &ObjectPoints);
};

#endif // LEASTSQUARE_H
