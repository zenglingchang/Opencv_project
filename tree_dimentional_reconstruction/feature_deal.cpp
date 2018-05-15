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


cv::Mat match_feature(Feature_Deal &a, Feature_Deal &b,vector<Point2f>& p1, vector<Point2f>& p2,cv::Rect rec1,cv::Rect rec2)
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
//    Mat mask=Mat::zeros(Size(rect.width/20+1,rect.height/20+1),CV_8UC1);
    for(auto match:temp)
        if((match.distance<5 * max(min_dist, 10.0f))&&rec1.contains(a.keypoints[match.queryIdx].pt)&&rec2.contains(b.keypoints[match.trainIdx].pt)/*
                mask.at<uchar>((a.keypoints[match.queryIdx].pt.y-rect.y)/20,(a.keypoints[match.queryIdx].pt.x-rect.x)/20)==0*/){
            matches.push_back(match);
//            mask.at<uchar>((a.keypoints[match.queryIdx].pt.y-rect.y)/20,(a.keypoints[match.queryIdx].pt.x-rect.x)/20)=1;
        }
    for(cv::DMatch match:matches){
            p1.push_back(a.keypoints[match.queryIdx].pt);
            p2.push_back(b.keypoints[match.trainIdx].pt);
    }
    cv::Mat outputimg;
    cv::drawMatches(a.image,a.keypoints,b.image,b.keypoints,matches,outputimg,cv::Scalar::all(-1),cv::Scalar::all(-1));
    return outputimg;
}

