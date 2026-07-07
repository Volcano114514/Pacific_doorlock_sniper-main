#include <QApplication>
#include "mainwindow.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    // 初始化 ROS2 客户端库
    rclcpp::init(argc, argv);

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    int ret = a.exec();

    // 可选：确保 ROS2 资源释放（析构函数已处理，这里也可调用）
    // rclcpp::shutdown(); // 实际上 MainWindow 析构时已调用，但若窗口未正常析构，可加

    return ret;
}