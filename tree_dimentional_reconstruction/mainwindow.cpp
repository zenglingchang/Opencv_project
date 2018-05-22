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
    for(int i=0;i<15;i++){
        filelist.push_back(string("C:\\Users\\77470\\Desktop\\Data_mining\\chess")+QString::number(i+1).toStdString()+".jpg");
    }
    struc.calculate_Camera(filelist);
}

void MainWindow::on_pushButton_2_clicked()
{
    std::vector<string> ImageNameList;
    for(int i=-7;i<8;i++)
        ImageNameList.push_back("C:\\Users\\77470\\Desktop\\Data_mining\\test"+QString::number(i+1).toStdString()+".jpg");
    struc.SetImage(ImageNameList);
    qDebug()<<"SetImage Complete!";
}

void MainWindow::on_pushButton_3_clicked()
{
    struc.BuildFace();
    qDebug()<<"BuildFace Complete!";
    struc.OutputToFile();
    qDebug()<<"Print To File Complete!";
}

void MainWindow::on_pushButton_4_clicked()
{
    struc.AddStruct(front,ui->checkBox->isChecked());
    qDebug()<<"AddStruct Complete!";
}


void MainWindow::on_pushButton_5_clicked()
{
    struc.AddStruct(mid,ui->checkBox->isChecked());
    qDebug()<<"AddStruct Complete!";
}
void MainWindow::on_pushButton_6_clicked()
{
    struc.AddStruct(back,ui->checkBox->isChecked());
    qDebug()<<"AddStruct Complete!";
}
