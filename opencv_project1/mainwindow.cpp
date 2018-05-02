#include "mainwindow.h"
#include "ui_mainwindow.h"
int X[]={-1,1,0,0,1,1,-1,-1};
int Y[]={0,0,-1,1,-1,1,-1};
std::stack<std::pair<int,int>> S_p;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->spinBox->setMinimum(0);
    ui->spinBox->setMaximum(255);
    ui->spinBox->setSingleStep(1);
    ui->Slider->setMinimum(0);
    ui->Slider->setMaximum(255);
    ui->Slider->setSingleStep(5);
    connect(ui->spinBox,SIGNAL(valueChanged(int)),ui->Slider,SLOT(setValue(int)));
    connect(ui->Slider,SIGNAL(valueChanged(int)),ui->spinBox,SLOT(setValue(int)));
    p1=std::pair<int,int>(190,533),p2=std::pair<int,int>(510,810);
}

MainWindow::~MainWindow()
{
    delete ui;
}

int MainWindow::d(cv::Vec3b a, cv::Vec3b b)
{
    return std::sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]));
}

void MainWindow::search_tree(cv::Mat &gray,cv::Mat &Fmat,std::pair<int,int> p,std::vector<std::pair<int,int>> &buffer)
{
    S_p.push(p);
    std::pair<int,int> temp;
    p1.first=cv::max(p1.first,0),p1.second=cv::max(p1.second,0);
    p2.first=cv::min(p2.first,image.rows),p2.second=cv::min(p2.second,image.cols);
    qDebug()<<p1.first<<" "<<p1.second<<" "<<p2.first<<" "<<p2.second;
    while(!S_p.empty()){
        temp=S_p.top();
        S_p.pop();
        for(int i=0;i<8;i++)
            if((temp.first+X[i]>p1.first&&temp.first+X[i]<p2.first)&&(temp.second+Y[i]>p1.second&&temp.second+Y[i]<p2.second)&&
                    //(int)cabs(color,(gray.at<uchar>(temp.first+X[i],temp.second+Y[i])/4)+3*((aver/num))/4)<ui->spinBox->value())
                    3*d(image.at<cv::Vec3b>(temp.first,temp.second),image.at<cv::Vec3b>(temp.first+X[i],temp.second+Y[i]))/4
                    +d(image.at<cv::Vec3b>(p.first,p.second),image.at<cv::Vec3b>(temp.first+X[i],temp.second+Y[i]))/4<ui->spinBox->value())
            {
                if(Fmat.at<uchar>(temp.first+X[i],temp.second+Y[i])!=255){
                    //qDebug()<<temp.first<<" "<<temp.second<<" "<<d(image.at<cv::Vec3b>(temp.first,temp.second),image.at<cv::Vec3b>(temp.first+X[i],temp.second+Y[i]))<<" "<<ui->spinBox->value();
                    Fmat.at<uchar>(temp.first+X[i],temp.second+Y[i])=255;
                    S_p.push(std::pair<int,int>(temp.first+X[i],temp.second+Y[i]));
                }
            }
       buffer.push_back(temp);
    }
}

void MainWindow::pooling(cv::Mat &src, cv::Mat &dst,int blockSize)
{
        dst=src;
        int half=blockSize/2;
        cv::Mat iimage;
        cv::integral(src,iimage,CV_32S);
        qDebug()<<"start!";
        for(int i=p1.first;i<p2.first;i++)
        {
            qDebug()<<i;
            int x1=cv::max(i-half,p1.first),x2=cv::min(i+half,p2.first-1);
            cv::Vec3b *data=dst.ptr<cv::Vec3b>(i);
            cv::Vec3i *idata1=iimage.ptr<cv::Vec3i>(x1);
            cv::Vec3i *idata2=iimage.ptr<cv::Vec3i>(x2);
            for(int j=p1.second;j<p2.second;j++)
            {
                int y1=cv::max(j-half,p1.second),y2=cv::min(j+half,p2.second-1),area=(y2-y1)*(x2-x1);
                if(data[j][0]==0&&data[j][1]==0&&data[j][2]==0){
                    for(int k=0;k<3;k++)
                        data[j][k]=(idata2[y2][k]+idata1[y1][k]-idata1[y2][k]-idata2[y1][k])/area;
                    //qDebug()<<i<<"  "<<j<<" "<<data[j][0]<<" "<<data[j][1]<<" "<<data[j][2]<<" "<<area;
                }
            }
        }
}

void MainWindow::cut(cv::Mat &src, cv::Mat &gray_cut)
{
    cv::Mat_<cv::Vec3b>::iterator itd=src.begin<cv::Vec3b>();
    cv::Mat_<uchar>::iterator itc=gray_cut.begin<uchar>();
    while(itc!=gray_cut.end<uchar>()){
        if(*itc==255)
            (*itd)[0]=0,(*itd)[1]=0,(*itd)[2]=0;
        itc++,itd++;
    }
}

void MainWindow::func(cv::Mat &result)
{
    cv::Mat Gcut=cv::imread("C:\\Users\\suning\\Desktop\\computer_vision\\opencv_1\\cut_tree.png"),
            temp=cv::imread("C:\\Users\\suning\\Desktop\\computer_vision\\opencv_1\\deal_tree.png");
    cut(temp,Gcut);
    result=temp;
    //pooling(temp,result,4);
    cv::namedWindow("image",CV_WINDOW_NORMAL);
    cv::imshow("image",result);
    cv::resizeWindow("image",1000,(int)1000.0*image.size().height/image.size().width);
}

void MainWindow::on_pushButton_clicked()
{
    cv::Mat temp;
    cv::cvtColor(image,temp,CV_BGR2RGB);
    cv::cvtColor(image,temp,CV_BGR2RGB);//opencv读取图片按照BGR方式读取，为了正常显示，所以将BGR转为RGB
    QImage showImage((const uchar*)temp.data,image.cols,image.rows,image.cols*image.channels(),QImage::Format_RGB888);
    showImage = showImage.scaled(ui->label->width(),ui->label->height(),Qt::KeepAspectRatio);
    ui->label->setPixmap(QPixmap::fromImage(showImage)); //将图片显示在QLabel上
}

void MainWindow::on_pushButton_2_clicked()
{
    std::vector<std::pair<int,int>> buffer;
    cv::Mat gray_img,flag_mat(cv::Size(image.cols,image.rows),CV_8UC1,cv::Scalar(0)),ima(cv::Size(image.cols,image.rows),CV_8UC3);
    cv::cvtColor(image,gray_img,CV_BGR2GRAY);
    search_tree(gray_img,flag_mat,std::pair<int,int>(ui->YEdit->text().toInt(),ui->XEdit->text().toInt()),buffer);
    for(auto p:buffer)
        for(int i=0;i<3;i++)
            ima.at<cv::Vec3b>(p.first,p.second)[i]=image.at<cv::Vec3b>(p.first,p.second)[i];
    cv::namedWindow("test",CV_WINDOW_NORMAL);
    cv::namedWindow("gray",CV_WINDOW_NORMAL);
    cv::namedWindow("image",CV_WINDOW_NORMAL);
    cv::imshow("test",flag_mat);
    cv::imshow("gray",gray_img);
    cv::imshow("image",ima);
    cv::resizeWindow("test",1000,(int)1000.0*image.size().height/image.size().width);
    cv::resizeWindow("gray",1000,(int)1000.0*image.size().height/image.size().width);
    cv::resizeWindow("image",1000,(int)1000.0*image.size().height/image.size().width);
}

void MainWindow::on_pushButton_3_clicked()
{
    cv::Mat result,iimage;
    func(result);
    for(int i=p1.first;i<p2.first;i++)
    {
        cv::Vec3b *data=result.ptr<cv::Vec3b>(i),
                *data1=image.ptr<cv::Vec3b>(i+5);
        for(int j=p1.second;j<p2.second;j++)
        {
            if(data[j][0]!=0||data[j][1]!=0||data[j][2]!=0){
                for(int k=0;k<3;k++)
                    data1[j-499][k]=data[j][k];
                //qDebug()<<i<<"  "<<j<<" "<<data[j][0]<<" "<<data[j][1]<<" "<<data[j][2]<<" "<<area;
            }
        }
    }
    int half=1;
    cv::integral(image,iimage,CV_32S);
    for(int i=p1.first;i<p2.first;i++)
    {
        qDebug()<<i;
        int x1=cv::max(i-half,p1.first),x2=cv::min(i+half,p2.first-1);
        cv::Vec3b *temp=result.ptr<cv::Vec3b>(i),*data=image.ptr<cv::Vec3b>(i+5);
        cv::Vec3i *idata1=iimage.ptr<cv::Vec3i>(x1+5);
        cv::Vec3i *idata2=iimage.ptr<cv::Vec3i>(x2+5);
        for(int j=p1.second;j<p2.second;j++)
        {
            int y1=cv::max(j-half,p1.second)-499,y2=cv::min(j+half,p2.second-1)-499,area=(y2-y1)*(x2-x1);
            if(temp[j][0]<2&&temp[j][1]<2&&temp[j][2]<2){
                for(int k=0;k<3;k++)
                   data[j-499][k]=(idata2[y2][k]+idata1[y1][k]-idata1[y2][k]-idata2[y1][k])/area;
                //qDebug()<<i<<"  "<<j<<" "<<data[j][0]<<" "<<data[j][1]<<" "<<data[j][2]<<" "<<area;
            }
        }
    }
}

void MainWindow::on_pushButton_4_clicked()
{
    cv::imwrite("C:\\Users\\suning\\Desktop\\computer_vision\\opencv_1\\save.jpg",image);
}
