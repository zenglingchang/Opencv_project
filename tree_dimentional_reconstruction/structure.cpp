#include "structure.h"

structure::structure()
{
    rects={cv::Rect(0,80,940,553-80),cv::Rect(0,86,947,547-86),
           cv::Rect(0,70,946,548-70),cv::Rect(0,67,884,567-67),
           cv::Rect(39,66,850-39,559-66),cv::Rect(93,33,758-93,553-33),
           cv::Rect(240,83,823-240,603-83),cv::Rect(218,22,765-218,586-22),
           cv::Rect(320,17,787-320,607-17),cv::Rect(258,19,787-258,597-19),
           cv::Rect(255,30,744-255,628-30),cv::Rect(205,16,781-205,641-16),
           cv::Rect(147,38,766-147,641-38),cv::Rect(20,46,861-20,562-46),
           cv::Rect(54,56,898-54,522-56),cv::Rect(0,62,950,557-62)};
}

void structure::SetImage(std::vector<string> ImageNameList)
{
    int ratio,i=0;
    cv::Mat image,tex;
    ImageNum=ImageNameList.size();
    Points2D_All.resize(ImageNum);
    FaceList.resize(ImageNum);
    Rotations.resize(ImageNum);
    Motions.resize(ImageNum);
    for(string ImageName:ImageNameList){
        image=cv::imread(ImageName);
        ratio=std::sqrt((image.cols*image.rows)/1000000);
        cv::resize(image,tex,cv::Size(1024,1024),0,0,INTER_AREA);
        cv::imwrite("D:\\Vs_project\\OpenGL_project\\Opengl-version1.0\\Opengl-version1.0\\texture" +
                    QString::number(i++).toStdString()+".jpg",tex);
        cv::resize(image,image,cv::Size(image.cols/ratio,image.rows/ratio),0,0,INTER_AREA);
        ImageList.push_back(image);
        FeatrueList.push_back(Feature_Deal(image));
    }
    rect=Rect(0,0,image.cols,image.rows);
}

void structure::calculate_Camera(std::vector<string> filenamelist)
{
    camera.calibrate(filenamelist);
}

void structure::init(int begin)
{
    if(ImageList.size()<2){
        qDebug()<<"至少需要两张图";
        return;
    }
    //特征匹配
    vector<cv::Point2f> p1,p2,temp1,temp2;
    cv::imshow("match_begin",match_feature(FeatrueList[begin],FeatrueList[begin+1],p1,p2,rects[begin],rects[begin+1]));
    cv::Mat mask;

    //本征矩阵及提取旋转平移矩阵
    cv::Mat E=cv::findEssentialMat(p1,p2,camera.focal_length,camera.principle_point,RANSAC,0.999,1.0,mask),R,T;
    recoverPose(E, p1, p2,R , T, camera.focal_length, camera.principle_point,mask);
    int j=0;
    for(cv::Mat_<uchar>::iterator it=mask.begin<uchar>(),itend=mask.end<uchar>();it!=itend;it++){
        if(*it==1){
            temp1.push_back(p1[j]);
            temp2.push_back(p2[j]);
        }
        j++;
    }
    Rotations[begin]=Mat::eye(3, 3, CV_64FC1);
    Motions[begin]=Mat::zeros(3,1,CV_64FC1);
    Rotations[begin+1]=R;
    Motions[begin+1]=T;

    //三角重建
    vector<Point3f> Points3D;
    reconstruct(Rotations[begin],Motions[begin],Rotations[begin+1],Motions[begin+1],temp1,temp2,Points3D);

    //添加映射
    for(int i=0;i<Points3D.size();i++){
        Points2D_All[begin].push_back(temp1[i]);
        Points2D_All[begin+1].push_back(temp2[i]);
        Points3D_All.push_back(Points3D[i]);
        PointsMap[TexturePoint(begin,temp1[i])]=Points3D_All.size()-1;
        PointsMap[TexturePoint(begin+1,temp2[i])]=Points3D_All.size()-1;
    }
}

void structure::Filter(vector<Point2f> &p1, vector<Point2f> &p2,cv::Mat &R,cv::Mat &T)
{
    vector<Point2f> temp1,temp2;
    cv::Mat mask;
    //本征矩阵
    cv::Mat E=cv::findEssentialMat(p1,p2,camera.focal_length,camera.principle_point,RANSAC,0.999,1.0,mask);
    recoverPose(E, p1, p2,R , T, camera.focal_length, camera.principle_point,mask);
    int j=0;
    for(cv::Mat_<uchar>::iterator it=mask.begin<uchar>(),itend=mask.end<uchar>();it!=itend;it++){
        if(*it==1){
            temp1.push_back(p1[j]);
            temp2.push_back(p2[j]);
        }
        j++;
    }
    p1=temp1;
    p2=temp2;
}

void structure::AddStruct(STATU statu)
{
    clear();
    int begin;
    if(statu==front)
        begin=0;
    else if(statu==mid)
        begin=(ImageNum-1)/2;
    else
        begin=ImageNum-2;
    init(begin);
    qDebug()<<"init Complete!";
    //以中间图像为开始
    //对中间图像后的图像进行匹配
    for(int i=begin+1;i<ImageNum-1;i++){
        cv::Mat r,T,R;
        vector<cv::Point2f> p1,p2,temp1,temp2,ImagePoints;
        vector<cv::Point3f> ObjectPoints;
        cv::imshow("match + "+QString::number(i).toStdString(),match_feature(FeatrueList[i],FeatrueList[i+1],p1,p2,rects[i],rects[i+1]));
        Filter(p1,p2,R,T);

        int Index,FindNumber=0;
        TexturePoint TempTexturePoint;
        for(int j=0;j<p1.size();j++){
            TempTexturePoint=TexturePoint(i,p1[j]);
            if(FindPoint3dIndex(TempTexturePoint,PointsMap,Index)){     //当前点如果存在空间点
                ObjectPoints.push_back(Points3D_All[Index]);            //加入三维点集
                ImagePoints.push_back(p2[j]);                           //加入图像点集
                Points2D_All[i+1].push_back(p2[j]);
                PointsMap[TexturePoint(i+1,p2[j])]=Index;
                qDebug()<<p1[j].x<<" "<<p1[j].y<<"  "<<Index;
                FindNumber++;
            }
            else{                                                       //
                temp1.push_back(p1[j]);
                temp2.push_back(p2[j]);
            }
        }
        qDebug()<<"Find 3D Points Complete!";
        if(FindNumber>4){
            solvePnPRansac(ObjectPoints,ImagePoints,camera.in_matrix,camera.distortion_coeffs,r,T);
            Rodrigues(r, R);
            qDebug()<<"match_feature "<<i+1<<" and "<<i+2<<" complete!";
        }
        else{
            qDebug()<<"match_feature "<<i+1<<" and "<<i+2<<" fail!";
            continue;
        }
        Rotations[i+1]=R;
        Motions[i+1]=T;
        vector<Point3f> Points3D;
        reconstruct(Rotations[i],Motions[i],Rotations[i+1],Motions[i+1],temp1,temp2,Points3D);

        Point3f zero(0.0,0.0,0.0);
        //添加映射
        for(int j=0;j<Points3D.size();j++){
            if(d(Points3D[j],zero)>14)
                continue;
            Points2D_All[i].push_back(temp1[j]);
            Points2D_All[i+1].push_back(temp2[j]);
            Points3D_All.push_back(Points3D[j]);
            PointsMap[TexturePoint(i,temp1[j])]=Points3D_All.size()-1;
            PointsMap[TexturePoint(i+1,temp2[j])]=Points3D_All.size()-1;
        }
    }

    //对中间图像前的图像进行匹配
    for(int i=begin;i>0;i--){
        cv::Mat r,T,R;
        vector<cv::Point2f> p1,p2,temp1,temp2,ImagePoints;
        vector<cv::Point3f> ObjectPoints;
        cv::imshow("match -"+QString::number(i).toStdString(),match_feature(FeatrueList[i-1],FeatrueList[i],p1,p2,rects[i-1],rects[i]));
        Filter(p1,p2,R,T);

        int Index,FindNumber=0;
        TexturePoint TempTexturePoint;
        for(int j=0;j<p1.size();j++){
            TempTexturePoint=TexturePoint(i,p2[j]);
            if(FindPoint3dIndex(TempTexturePoint,PointsMap,Index)){     //当前点如果存在空间点
                ObjectPoints.push_back(Points3D_All[Index]);            //加入三维点集
                ImagePoints.push_back(p1[j]);                           //加入图像点集
                Points2D_All[i-1].push_back(p1[j]);
                PointsMap[TexturePoint(i-1,p1[j])]=Index;
                qDebug()<<p1[j].x<<" "<<p1[j].y<<"  "<<Index;
                FindNumber++;
            }
            else{                                                       //
                temp1.push_back(p1[j]);
                temp2.push_back(p2[j]);
            }
        }
        qDebug()<<"Find 3D Points Complete!";
        if(FindNumber>4){
            solvePnPRansac(ObjectPoints,ImagePoints,camera.in_matrix,camera.distortion_coeffs,r,T);
            Rodrigues(r, R);
            qDebug()<<"match_feature "<<i+1<<" and "<<i<<" complete!";
        }
        else{
            qDebug()<<"match_feature "<<i+1<<" and "<<i<<" fail!";
            continue;
        }
        Rotations[i-1]=R;
        Motions[i-1]=T;
        vector<Point3f> Points3D;
        reconstruct(Rotations[i-1],Motions[i-1],Rotations[i],Motions[i],temp1,temp2,Points3D);

        Point3f zero(0.0,0.0,0.0);
        //添加映射
        for(int j=0;j<Points3D.size();j++){
            if(d(Points3D[j],zero)>14)
                continue;
            Points2D_All[i-1].push_back(temp1[j]);
            Points2D_All[i].push_back(temp2[j]);
            Points3D_All.push_back(Points3D[j]);
            PointsMap[TexturePoint(i-1,temp1[j])]=Points3D_All.size()-1;
            PointsMap[TexturePoint(i,temp2[j])]=Points3D_All.size()-1;
        }
    }
//    for(auto Values:PointsMap)
//        qDebug()<<Values.first.TexId<<Values.first.position.x<<Values.first.position.y<<Values.second;
}

void structure::BuildFace()
{
    for(int i=1;i<ImageNum-1;i++){
        Delaunay delaunay(rect);
        delaunay.insert(Points2D_All[i]);
        delaunay.GetFace(FaceList[i]);
    }
}

void structure::OutputToFile()
{
    map<Face,int> FaceMap;
    ofstream file("D:\\qt_project\\tree_dimensional_reconstruction\\tree_dimentional_reconstruction\\model.obj",std::ios::trunc);
    ofstream file2("D:\\qt_project\\tree_dimensional_reconstruction\\tree_dimentional_reconstruction\\data.txt",std::ios::trunc);
    if(file&&file2){
        int xx=0;
        for(Point3f p:Points3D_All){
            file<<xx++<<" v "<<p.x<<" "<<p.y<<" "<<p.z<<endl;
        }
        for(int num=1;num<ImageNum-1;num++)
            for(int i=0;i<FaceList[num].size();){
                Face TempFace(PointsMap[TexturePoint(num,Points2D_All[num][FaceList[num][i]])],
                        PointsMap[TexturePoint(num,Points2D_All[num][FaceList[num][i+1]])],
                        PointsMap[TexturePoint(num,Points2D_All[num][FaceList[num][i+2]])]);
                qDebug()<<TempFace.F[0]<<TempFace.F[1]<<TempFace.F[2];
                if(FaceMap.find(TempFace)!=FaceMap.end()){
                    qDebug()<<"find face!";
                    i+=3;
                    continue;
                }
                else
                    FaceMap[TempFace]=1;
                if((d(Points3D_All[TempFace.F[0]],Points3D_All[TempFace.F[1]])>1)||
                        (d(Points3D_All[TempFace.F[1]],Points3D_All[TempFace.F[2]])>1)||
                        (d(Points3D_All[TempFace.F[0]],Points3D_All[TempFace.F[2]])>1))
                    continue;
                for(int k=0;k<3;k++){
                    int index=TempFace.F[k];
                    file2<<Points3D_All[index].x<<" "<<-Points3D_All[index].y<<" "<<Points3D_All[index].z<<" "<<num<<" "
                        <<Points2D_All[num][FaceList[num][i+k]].x/rect.width<<" "<<1-Points2D_All[num][FaceList[num][i+k]].y/rect.height<<endl;

            }
            file<<"f "<<PointsMap[TexturePoint(num,Points2D_All[num][FaceList[num][i]])]
                    <<" "<<PointsMap[TexturePoint(num,Points2D_All[num][FaceList[num][i+1]])]
                    <<" "<<PointsMap[TexturePoint(num,Points2D_All[num][FaceList[num][i+2]])]<<endl;
            i+=3;
        }
        file.close();
        file2.close();
    }
    else
        qDebug()<<"Open file error!";
}

void structure::calc3Dpts(vector<Point2f> &pr, vector<Point2f> &pl, Mat &MR, Mat &ML, vector<Point3f> &pts)
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

void structure::reconstruct(Mat &RLeft, Mat &Tleft, Mat &Rright, Mat &Tright, std::vector<Point2f> &Points2Dleft, std::vector<Point2f> &Points2Dright, std::vector<Point3f> &Points3D)
{
    cv::Mat proj1(3,4,CV_64FC1),proj2(3,4,CV_64FC1);
    RLeft.convertTo(proj1(Range(0,3),Range(0,3)),CV_64FC1);
    Tleft.convertTo(proj1.col(3),CV_64FC1);
    Rright.convertTo(proj2(Range(0,3),Range(0,3)),CV_64FC1);
    Tright.convertTo(proj2.col(3),CV_64FC1);
    proj1=camera.in_matrix*proj1;
    proj2=camera.in_matrix*proj2;
    calc3Dpts(Points2Dleft,Points2Dright,proj1,proj2,Points3D);
}

bool FindPoint3dIndex(TexturePoint &p, map<TexturePoint, int> &PointsMap, int &Index)
{
    auto it=PointsMap.find(p);
    if(it!=PointsMap.end()){
        Index=it->second;
        return true;
    }
    return false;
}

float d(Point2f &p1, Point2f &p2)
{
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

float d(Point3f &p1, Point3f &p2)
{
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
}
