#ifndef ELEPHANT_PORT_H
#define ELEPHANT_PORT_H
#include"help.h"

class elephant_port
{
public:
    cv::Mat img;
    cv::Point2f left_top,right_top;
    double len_left,len_right;
    cv::Point2f center;
    cv::Mat mask;
    elephant_port();
    void Grow(int detaleft=0,int detaright=0);
    int d(cv::Vec3b a,cv::Vec3b b);
};
std::vector<int> sort(std::vector<std::pair<double,double>> &len);
void solu(std::vector<std::pair<double,double>> &data,std::vector<int> &number,int left,double sum,double &min,std::vector<int> &answer);
#endif // ELEPHANT_PORT_H
