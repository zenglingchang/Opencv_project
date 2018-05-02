#include "feature.h"

feature::feature(cv::Mat img)
{
    image=img;
    Detector->detect(image,keypoints);
    Detector->compute(image,keypoints,descriptors);
}

void match_feature(feature &a, feature &b)
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
    for(auto a:temp)
        if(a.distance<4 * std::max(min_dist, 10.0f))
            matches.push_back(a);
    cv::Mat outputimg;
    cv::drawMatches(a.image,a.keypoints,b.image,b.keypoints,matches,outputimg,cv::Scalar::all(-1),cv::Scalar::all(-1));
    cv::imshow("output",outputimg);
}
