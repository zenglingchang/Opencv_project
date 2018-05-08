#include "feature_deal.h"

Feature_Deal::Feature_Deal(cv::Mat &ima)
{
//    cv::cvtColor(ima,ima,CV_RGB2GRAY);c
    ratio=std::sqrt((ima.cols*ima.rows)/500000);
    cv::resize(ima,ima,cv::Size(ima.cols/ratio,ima.rows/ratio),0,0,INTER_AREA);
    image=ima;
//    cv::equalizeHist(image,result);
//    cv::imshow("src",image);
//    cv::imshow("dst",result);
//    cv::Mat imageEnhance;
//    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
//    cv::filter2D(image, imageEnhance, CV_8UC3, kernel);
//    Detector->detect(imageEnhance,keypoints);
    Detector->detect(image,keypoints);
    Detector->compute(image,keypoints,descriptors);
}


void match_feature(Feature_Deal &a, Feature_Deal &b,vector<Point2f>& p1, vector<Point2f>& p2,cv::Rect rect)
{
    //构造匹配器
    cv::BFMatcher Dmatch(cv::NORM_L2);
    std::vector<cv::DMatch> temp,matches;
    std::vector<std::vector<cv::DMatch>> matches2;
    Dmatch.knnMatch(a.descriptors,b.descriptors,matches2,2);
    double ratio=0.8;
    float min_dist = FLT_MAX;
    for(auto it=matches2.begin();it!=matches2.end();it++){
        if((*it)[0].distance/(*it)[1].distance<ratio){
            temp.push_back((*it)[0]);
            float dist=(*it)[0].distance;
            if(dist<min_dist) min_dist=dist;
        }
    }
    Mat mask=Mat::zeros(Size(rect.width/20+1,rect.height/20+1),CV_8UC1);
    for(auto match:temp)
        if((match.distance<5 * max(min_dist, 10.0f))&&rect.contains(a.keypoints[match.queryIdx].pt)&&
                mask.at<uchar>((a.keypoints[match.queryIdx].pt.y-rect.y)/20,(a.keypoints[match.queryIdx].pt.x-rect.x)/20)==0){
            matches.push_back(match);
            mask.at<uchar>((a.keypoints[match.queryIdx].pt.y-rect.y)/20,(a.keypoints[match.queryIdx].pt.x-rect.x)/20)=1;
        }
    for(cv::DMatch match:matches){
            p1.push_back(a.keypoints[match.queryIdx].pt);
            p2.push_back(b.keypoints[match.trainIdx].pt);
    }
    cv::Mat outputimg;
    cv::drawMatches(a.image,a.keypoints,b.image,b.keypoints,matches,outputimg,cv::Scalar::all(-1),cv::Scalar::all(-1));
    cv::imshow("output",outputimg);
}
void calc3Dpts(vector<Point2f> &pr, vector<Point2f> &pl,Mat &MR,Mat &ML, vector<Point3f> &pts)
{

    Mat cof(4,4,CV_64F), z;
    double u1,v1,u2,v2;
    int len, i, j;
    len = pr.size();
    pts.reserve(len);
    for (i=0; i < len; i++)
    {
        u1=pr[i].x;
        v1=pr[i].y;
        u2=pl[i].x;
        v2=pl[i].y;
        *((double*)cof.data)=*((double*)MR.data+8)*u1-*((double*)MR.data);
        *((double*)cof.data+1)=*((double*)MR.data+9)*u1-*((double*)MR.data+1);
        *((double*)cof.data+2)=*((double*)MR.data+10)*u1-*((double*)MR.data+2);
        *((double*)cof.data+3)=*((double*)MR.data+11)*u1-*((double*)MR.data+3);
        *((double*)cof.data+4)=*((double*)MR.data+8)*v1-*((double*)MR.data+4);
        *((double*)cof.data+5)=*((double*)MR.data+9)*v1-*((double*)MR.data+5);
        *((double*)cof.data+6)=*((double*)MR.data+10)*v1-*((double*)MR.data+6);
        *((double*)cof.data+7)=*((double*)MR.data+11)*v1-*((double*)MR.data+7);
        *((double*)cof.data+8)=*((double*)ML.data+8)*u2-*((double*)ML.data);
        *((double*)cof.data+9)=*((double*)ML.data+9)*u2-*((double*)ML.data+1);
        *((double*)cof.data+10)=*((double*)ML.data+10)*u2-*((double*)ML.data+2);
        *((double*)cof.data+11)=*((double*)ML.data+11)*u2-*((double*)ML.data+3);
        *((double*)cof.data+12)=*((double*)ML.data+8)*v2-*((double*)ML.data+4);
        *((double*)cof.data+13)=*((double*)ML.data+9)*v2-*((double*)ML.data+5);
        *((double*)cof.data+14)=*((double*)ML.data+10)*v2-*((double*)ML.data+6);
        *((double*)cof.data+15)=*((double*)ML.data+11)*v2-*((double*)ML.data+7);

    //  cout << "cof:" << endl << cof << endl;
        SVD::solveZ(cof, z);
        for (j=0;j<3;j++) *((double*)z.data+j)/=*((double*)z.data+3);
        pts.push_back(Point3f(float(*((double*)z.data)), float(*((double*)z.data+1)), float(*((double*)z.data+2))));
    }
}
