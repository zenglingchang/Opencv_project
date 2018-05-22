#include "leastsquare.h"

LeastSquare::LeastSquare()
{

}



void LeastSquare::init(Mat &r, Mat &T,Mat &K)
{
    for(int i=0;i<6;i++){
        Grad[i]=0;
        GradRatio[i]=0.001;
    }
    dx=0.00000001;
    GradValid=false;
    SetRT(r,T,K);
}

double LeastSquare::cost(Mat &Proj, vector<Point2f> &ImagePoints, vector<Point3f> &ObjectPoints)
{
    double count=0,Size=ImagePoints.size(),Dx=0,Dy=0;
    cv::Mat ObjectPoint(4,1,CV_64FC1),ImagePointTemp;
    for(int i=0;i<ImagePoints.size();i++){
        *((double*)ObjectPoint.data)=ObjectPoints[i].x;
        *((double*)ObjectPoint.data+1)=ObjectPoints[i].y;
        *((double*)ObjectPoint.data+2)=ObjectPoints[i].z;
        *((double*)ObjectPoint.data+3)=1.0;
        ImagePointTemp=Proj*ObjectPoint;
        Dx=ImagePoints[i].x-*((double*)ImagePointTemp.data)/(*((double*)ImagePointTemp.data+2));
        Dy=ImagePoints[i].y-*((double*)ImagePointTemp.data+1)/(*((double*)ImagePointTemp.data+2));
        count=(Dx*Dx+Dy*Dy)/Size;
    }
    return count;
}

void LeastSquare::solution(vector<Point2f> &ImagePoints, vector<Point3f> &ObjectPoints)
{
    int count=0,flag[6]={0,0,0,0,0,0};//计算完的梯度方向数，梯度方向数完成Flag
    double grad=0,Cost2=0;
    cv::Mat proj(3,4,CV_64FC1),R,tempr=r,tempT=T;
    Rodrigues(r, R);
    R.convertTo(proj(Range(0,3),Range(0,3)),CV_64FC1);
    T.convertTo(proj.col(3),CV_64FC1);
    proj=K*proj;
    Cost=cost(proj,ImagePoints,ObjectPoints);
    qDebug()<<"Cost : "<<Cost;
    for(int i=0;i<6;i++){
        grad=GetGrad(i,ImagePoints,ObjectPoints);
        GradRatio[i]=std::fabs(Switch(r,T,0,i)/grad)*0.01;
    }
    while(count<6){
        for(int i=0;i<6;i++){
            if(flag[i]==0){
                grad=GetGrad(i,ImagePoints,ObjectPoints);
                qDebug()<<"i="<<i<<"grad="<<grad;
                Switch(tempr,tempT,(-1.0)*grad*GradRatio[i],i);
                Rodrigues(tempr, R);
                R.convertTo(proj(Range(0,3),Range(0,3)),CV_64FC1);
                tempT.convertTo(proj.col(3),CV_64FC1);
                proj=K*proj;
                Cost2=cost(proj,ImagePoints,ObjectPoints);
                qDebug()<<"Cost:"<<Cost<<"Cost2 :"<<Cost2<<"Ratio:"<<GradRatio[i];
                if(Cost2<=Cost){
                    if(Cost-Cost2<0.000001){
                        flag[i]=1;
                        count++;
                    }
                    qDebug()<<"Swap!";
                    Cost=Cost2;
                    GradClear();
                }
                else{
                    qDebug()<<"No Swap!";
                    Switch(tempr,tempT,(1.0)*grad*GradRatio[i],i);
                    GradRatio[i]*=0.75;
                }
            }
        }
    }

}

double LeastSquare::Switch(Mat &tempr, Mat &tempT, double dx, int i)
{
    double temp;
    switch(i){
    case 0:*((double*)tempT.data)+=dx;
        temp=*((double*)tempT.data);
            qDebug()<<*((double*)tempT.data);
        break;
    case 1:*((double*)tempT.data+1)+=dx;
        temp=*((double*)tempT.data+1);
        qDebug()<<*((double*)tempT.data+1);
        break;
    case 2:*((double*)tempT.data+2)+=dx;
        temp=*((double*)tempT.data+2);
        qDebug()<<*((double*)tempT.data+2);
        break;
    case 3:*((double*)tempr.data)+=dx;
        temp=*((double*)tempr.data);
        qDebug()<<*((double*)tempr.data);
        break;
    case 4:*((double*)tempr.data+1)+=dx;
        temp=*((double*)tempr.data+1);
        qDebug()<<*((double*)tempr.data+1);
        break;
    case 5:*((double*)tempr.data+2)+=dx;
        temp=*((double*)tempr.data+2);
        qDebug()<<*((double*)tempr.data+2);
        break;
    }
    return temp;
}


double LeastSquare::GetGrad(int &i, vector<Point2f> &ImagePoints, vector<Point3f> &ObjectPoints)
{
    if(Grad[i]!=0)
        return Grad[i];
    cv::Mat tempr=r,tempT=T,proj(3,4,CV_64FC1),R;
    Switch(tempr,tempT,dx,i);
    Rodrigues(tempr, R);
    R.convertTo(proj(Range(0,3),Range(0,3)),CV_64FC1);
    tempT.convertTo(proj.col(3),CV_64FC1);
    Switch(tempr,tempT,-dx,i);
    proj=K*proj;
    double Cost2=cost(proj,ImagePoints,ObjectPoints);
    qDebug()<<"Cost2:"<<Cost2;
    Grad[i]=(Cost2-Cost)/dx;
    return Grad[i];
}

bool LeastSquare::operator()(Mat &r, Mat &T, Mat &K, vector<Point2f> &ImagePoints, vector<Point3f> &ObjectPoints)
{
    init(r,T,K);
    solution(ImagePoints,ObjectPoints);
    r=this->r;
    T=this->T;
}
