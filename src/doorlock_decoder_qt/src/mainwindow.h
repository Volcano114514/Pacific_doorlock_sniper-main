#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// 视频源枚举：与下拉框顺序一一对应，对齐launch话题架构
enum VideoSource
{
    SOURCE_MQTT_LOCAL = 0,    // 第1路：MQTT CustomByteBlock 解码画面 /decoded_image/local
    SOURCE_SHARK_UDP = 1,     // 第2路：UDP Shark 原生高清图传 /decoded_image/shark
    SOURCE_LOCAL_CAM_RAW = 2  // 第3路：本地大恒相机原生画面 /image_raw
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void callbackLocalCam(const sensor_msgs::msg::Image::SharedPtr msg);
    void callbackSharkImg(const sensor_msgs::msg::Image::SharedPtr msg);
    void callbackMqttImg(const sensor_msgs::msg::Image::SharedPtr msg);
    void callbackRefereeStatus(const std_msgs::msg::String::SharedPtr msg);

private slots:
    void slotSwitchVideoSource(int idx);
    void slotUpdateDisplay();

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node_;

    void drawCrosshair(cv::Mat &img);

    // 三路订阅器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_local_cam_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_shark_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_mqtt_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_game_status_;

    QTimer refresh_timer_;
    VideoSource current_source_;

    // 三路画面缓存
    cv::Mat mat_local_cam_;
    cv::Mat mat_shark_;
    cv::Mat mat_mqtt_;

    QString referee_text_;
};

#endif // MAINWINDOW_H
