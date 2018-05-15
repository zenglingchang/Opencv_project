#ifndef STRUCTURE_H
#define STRUCTURE_H
#include"help.h"
#include"feature_deal.h"
#include"Camera.h"
#include"delaunay.h"
enum STATU{front,mid,back};
struct Face{
    int F[3];
    Face(){}
    Face(int &a,int &b,int &c){
        F[0]=a,F[1]=b,F[2]=c;
        for(int i=0;i<3;i++)
            for(int j=i+1;j<3;j++)
                if(F[i]>F[j])
                    swap(F[i],F[j]);
    }
    bool operator <(const Face &b) const{
        return (this->F[0]==b.F[0])?
                ((this->F[1]==b.F[1])?
                (this->F[2]<b.F[2]):(this->F[1]<b.F[1]))
                :(this->F[0]<b.F[0]);
    }
};
struct TexturePoint{
    unsigned short TexId;      //所在纹理图序号
    Point2f position;           //纹理坐标
    TexturePoint(){}
    TexturePoint(unsigned short Id,Point2f Pos){
        TexId=Id;
        position=Pos;
    }
    bool operator <(const TexturePoint &b) const {
        return (this->TexId==b.TexId)?
                    ((this->position.x==b.position.x)?
                         (this->position.y<b.position.y):(this->position.x<b.position.x))
                  :(this->TexId<b.TexId);
    }
};

class structure
{
    int ImageNum;                                       //图像数
    Camera camera;                                      //相机内参
    cv::Rect rect;                                      //重建范围
    map<TexturePoint,int> PointsMap;                    //纹理坐标和空间点Index的映射
    std::vector<std::vector<Point2f>> Points2D_All;     //纹理坐标集合
    std::vector<cv::Point3f> Points3D_All;              //空间坐标集合
    std::vector<cv::Mat> ImageList;                     //图像List
    std::vector<Feature_Deal> FeatrueList;              //特诊List
    std::vector<cv::Mat> Rotations;                     //旋转矩阵List
    std::vector<cv::Mat> Motions;                       //平移矩阵List
    std::vector<std::vector<int>> FaceList;             //三角面片List
    std::vector<cv::Rect> rects;                        //最小外包矩形List
public:
    structure();
    void SetImage(std::vector<string> ImageNameList);           //输入图像
    void calculate_Camera(std::vector<string> filenamelist);    //标定相机
    void init(int begin);                                                //初始化，以第一幅图为标准建立坐标系
    void Filter(vector<Point2f> &p1,vector<Point2f> &p2,cv::Mat &R,cv::Mat &T);       //利用RANSAC过滤
    void AddStruct(STATU statu);                                           //增量添加空间点
    void BuildFace();                                           //利用Delaunay算法构造三角面片
    void OutputToFile();
    void clear(){
        for(std::vector<Point2f> Points:Points2D_All)
            Points.clear();
        Points3D_All.clear();
        PointsMap.clear();
        for(std::vector<int> faces:FaceList)
            faces.clear();

    }
    //利用三角重建计算三维点坐标
    void calc3Dpts(vector<Point2f> &pr, vector<Point2f> &pl,Mat &MR,Mat &ML, vector<Point3f> &pts);
    //三角重建
    void reconstruct(Mat &RLeft,Mat &Tleft,Mat &Rright,Mat &Tright,std::vector<cv::Point2f> &Points1,std::vector<cv::Point2f> &Points2,std::vector<cv::Point3f> &Points3D);
};

bool FindPoint3dIndex(
        TexturePoint &p,
        map<TexturePoint,int> &PointsMap,
        int &Index);
float d(Point2f &p1,Point2f &p2);
float d(Point3f &p1,Point3f &p2);
#endif // STRUCTURE_H
