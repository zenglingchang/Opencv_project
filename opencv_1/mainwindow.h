#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include"histogram.h"
#include"help.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    cv::Mat image=cv::imread("C:\\Users\\77470\\Desktop\\computer_vision\\opencv_1\\tree.jpg");
    int d(cv::Vec3b a,cv::Vec3b b);
    std::pair<int,int> p1,p2;
protected:
    void search_tree(cv::Mat &gray, cv::Mat &Fmat, std::pair<int,int> p, std::vector<std::pair<int,int>> &buffer);
    void pooling(cv::Mat &src,cv::Mat &dst,int blockSize);//0值池化函数，blockSize为块矩阵维度
    void cut(cv::Mat &src, cv::Mat &gray_cut);//要求cut与src等大
    void func(cv::Mat &result);
private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
