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
    string name="1.jpg";
    for(int i=0;i<5;i++){
        name[0]='1'+i;
        filelist.push_back(string("C:\\Users\\77470\\Desktop\\Data_mining\\chess")+name);
    }

    //初始化
    int len=29;                             //棋盘每格为29mmX29mm
    cv::Mat image;                          //存储图像
    std::vector<cv::Point2f> imageCorners;  //存储二维角点
    std::vector<cv::Point3f> objectCorners; //存储三维角点
    cv::Size boardSize(9,6);                //9*6的棋盘
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    //棋盘三维坐标
    for(int i=0;i<boardSize.height;i++)
        for(int j=0;j<boardSize.width;j++)
            objectCorners.push_back(cv::Point3f(i*len,j*len,0.0f));

    for(int i=0;i<filelist.size();i++){
        image=cv::imread(filelist[i],0);
        double ratio=std::sqrt((image.cols*image.rows)/500000);
        qDebug()<<ratio<<"  "<<image.cols<<"  "<<image.rows;
        cv::resize(image,image,cv::Size(image.cols/ratio,image.rows/ratio),0,0,INTER_AREA);
        //棋盘格角点检测
        qDebug()<<i;
        bool found=cv::findChessboardCorners(image,boardSize,imageCorners,CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        //亚精度像素角点检测
        if(found){
        cv::cornerSubPix(
                    image,
                    imageCorners,
                    cv::Size(5,5),          //搜索边长大小的一半
                    cv::Size(-1,-1),
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 30, 0.1));
            imagePoints.push_back(imageCorners);
            objectPoints.push_back(objectCorners);
        }
        qDebug()<<i;
    }
    //获取相机参数和旋转平移向量
    std::vector<cv::Mat> rotation_vectors,translation_vector;
    cv::calibrateCamera(objectPoints,imagePoints,image.size(),camera.in_matrix,camera.distortion_coeffs,rotation_vectors,translation_vector);
    for(int i=0;i<5;i++){
        vector<cv:Point3f> tempPoints=objectPoints[i];
        projectPoints(tempPoints,rotation_vectors[i],translation_vector[i],camera.in_matrix,camera.distortion_coeffs,);
    }


//    bool found=cv::findChessboardCorners(chess,boardSize,imageCorners);
//    cv::drawChessboardCorners(chess,boardSize,imageCorners,found);
//    cv::imshow("chess",chess);
}

void MainWindow::on_pushButton_2_clicked()
{
    cv::Mat im1=cv::imread("C:\\Users\\77470\\Desktop\\Data_mining\\test1.jpg"),
            im2=cv::imread("C:\\Users\\77470\\Desktop\\Data_mining\\test2.jpg");
    Feature_Deal test1(im1),test2(im2);
    match_feature(test1,test2);
}
