#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

//void MainWindow::on_pushButton_2_clicked()
//{
//    cv::Mat image=cv::imread("C:\\Users\\suning\\Desktop\\Data_mining\\chessboard.png");
//    int nc = image.channels();

//    int nWidthOfROI = 90;

//    for (int j=0;j<image.rows;j++)
//    {
//        uchar* data= image.ptr<uchar>(j);
//        for(int i=0;i<image.cols*nc;i+=nc)
//        {
//            if( (i/nc/nWidthOfROI + j/nWidthOfROI) % 2)
//            {
//                // bgr
//                data[i/nc*nc + 0] = 255 ;
//                data[i/nc*nc + 1] = 255 ;
//                data[i/nc*nc + 2] = 255 ;
//            }
//        }
//    }
//    cv::imshow("test",image);
//    qDebug()<<"打印成功";
//    cv::imwrite("C:\\Users\\77470\\Desktop\\Data_mining\\chessboard.png",image);
//}

void MainWindow::on_pushButton_clicked()
{
    //棋盘图路径
    std::vector<string> filelist;
    for(int i=0;i<12;i++){
        filelist.push_back(string("C:\\Users\\77470\\Desktop\\Data_mining\\chess")+QString::number(i+1).toStdString()+".jpg");
    }
    camera.calibrate(filelist);
}

void MainWindow::on_pushButton_2_clicked()
{
    cv::Mat im1=cv::imread("C:\\Users\\77470\\Desktop\\Data_mining\\test2.jpg"),
            im2=cv::imread("C:\\Users\\77470\\Desktop\\Data_mining\\test1.jpg"),tex;
    cv::resize(im1,tex,cv::Size(1024,1024),0,0,INTER_AREA);
    cv::imwrite("D:\\Vs_project\\OpenGL_project\\Opengl-version1.0\\Opengl-version1.0\\texture.jpg",tex);
    Feature_Deal test1(im1),test2(im2);
    cv::Rect rec(226,126,570-226,457-126);
    cv::rectangle(im1,rec,cv::Scalar::all(255));
    cv::imshow("im1",im1);
    std::vector<cv::Point2f> p1,p2;
    match_feature(test1,test2,p1,p2,rec);
    //通过内参矩阵求焦距及光心
    double focal_length = 0.5*(camera.in_matrix.at<double>(0) + camera.in_matrix.at<double>(4));
    Point2d principle_point(camera.in_matrix.at<double>(2), camera.in_matrix.at<double>(5));
    qDebug()<<"f: "<<focal_length<<" center: "<<principle_point.x<<" , "<<principle_point.y;
    Mat mask;
    cv::Mat E=cv::findEssentialMat(p1,p2,focal_length,principle_point,RANSAC,0.999,1.0,mask);
    double feasible_count = countNonZero(mask);
    qDebug() << (int)feasible_count << " -in- " << p1.size();
    cv::Mat R,T;
    int pass_count = recoverPose(E, p1, p2,R , T, focal_length, principle_point, mask);
    std::vector<cv::Point2f> temp1,temp2;
    int j=0,i=0;
    for(cv::Mat_<uchar>::iterator it=mask.begin<uchar>(),itend=mask.end<uchar>();it!=itend;it++){
        if(*it==1)
        {
            temp1.push_back(p1[j]);
            temp2.push_back(p2[j]);
            qDebug()<<"p1: ( "<<p1[j].x<<" , "<<p1[j].y<<" ) p2: ( "
               <<p2[j].x<<" , "<<p2[j].y<<" )";
        }
        j++;
    }
    qDebug()<<"pass_rate: "<<((double)pass_count) / feasible_count;
    //两个相机的投影矩阵[R T]，triangulatePoints只支持float型
    Mat proj1(3, 4, CV_32FC1);
    Mat proj2(3, 4, CV_32FC1);

    proj1(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_32FC1);
    proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);

    R.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
    T.convertTo(proj2.col(3), CV_32FC1);

    Mat fK,structure;
    camera.in_matrix.convertTo(fK, CV_32FC1);
    proj1 = fK*proj1;
    proj2 = fK*proj2;
    vector<cv::Point3f> Points3d;
    //三角化重建
    triangulatePoints(proj1, proj2, temp1, temp2, structure);
    i=0;
    float x,y,z,w;
    for(cv::Mat_<float>::iterator it=structure.begin<float>(),itend=structure.end<float>();it!=itend;){
        x=*(it),y=*(it+1),z=*(it+2),w=*(it+3);
        it+=4;
        qDebug()<<x/w<<", "<<y/w<<", "<<z/w<<", "<<temp1[i].x/im1.cols<<","<<1-temp1[i].y/im1.rows<<",";
        i++;
        Points3d.push_back(cv::Point3f(x/w,y/w,z/w));
    }
    qDebug()<<i;
    fstream file("D:\\qt_project\\tree_dimensional_reconstruction\\tree_dimentional_reconstruction\\model.txt");
    if(file){
        for(cv::Point3f pt:Points3d)
            file<<"v "<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
        for(cv::Point2f pt:temp1)
            file<<"uv "<<pt.x/im1.cols<<" "<<1-pt.y/im1.rows<<endl;
        file.close();
    }
    else
        qDebug()<<"file does not exist!";
    Mat dst;
    Delaunay delaunay(rec);
    delaunay.insert(temp1);
    delaunay.drawDelaunay(im1,dst,Scalar::all(255));
    cv::imshow("dst",dst);
}
