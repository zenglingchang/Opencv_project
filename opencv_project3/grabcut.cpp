#include "grabcut.h"

GrabCut::GrabCut()
{

}

cv::Mat GrabCut::solution(cv::Mat image, cv::Rect rectangle)
{
    cv::grabCut(image,
                result,
                rectangle,
                bgModel,
                fgModel,
                5,
                cv::GC_INIT_WITH_RECT);
    cv::compare(result,cv::GC_PR_FGD,result,cv::CMP_EQ);
    result=result&1;
    //生成输出图像
    cv::Mat foreground(image.size(),CV_8UC3,
                       cv::Scalar(255,255,255));
    image.copyTo(foreground,result);
    return foreground;
}
