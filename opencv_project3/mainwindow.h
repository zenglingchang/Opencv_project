#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include"help.h"
#include"histogram.h"
#include"grabcut.h"
#include"feature.h"
#include"elephant_port.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
