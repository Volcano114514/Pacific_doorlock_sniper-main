#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QTimer>
#include <QImage>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// 两路视频源枚举
enum VideoSource
{
    SOURCE_SHARK_UDP,
    SOURCE_CUSTOM_VIDEO
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void update_display();
    void on_source_changed(int idx);

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_shark;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_custom;
    std::thread spin_thread;
    QTimer refresh_timer;
    rclcpp::Time last_img_time_;  // 记录最后收到图像的时间

    VideoSource current_source = SOURCE_SHARK_UDP;
    QImage display_img;

    void cb_shark(const sensor_msgs::msg::Image::SharedPtr msg);
    void cb_custom(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif