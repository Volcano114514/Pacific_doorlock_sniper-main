#include <QApplication>
#include "mainwindow.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    int ret = a.exec();
    rclcpp::shutdown();
    return ret;
}
