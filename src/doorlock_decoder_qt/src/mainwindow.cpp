#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QPixmap>
#include <QComboBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Video Display");

    // ROS节点
    node_ = rclcpp::Node::make_shared("qt_video_display");

    // 设置 QoS 为 BEST_EFFORT，匹配发布者
    rclcpp::QoS qos_best_effort(rclcpp::KeepLast(10));
    qos_best_effort.best_effort();

    // 订阅 Shark UDP 解码画面
    sub_shark = node_->create_subscription<sensor_msgs::msg::Image>(
        "/decoded_image/shark", qos_best_effort,
        std::bind(&MainWindow::cb_shark, this, std::placeholders::_1));

    // 订阅自定义客户端视频
    sub_custom = node_->create_subscription<sensor_msgs::msg::Image>(
        "/decoded_image/custom", qos_best_effort,
        std::bind(&MainWindow::cb_custom, this, std::placeholders::_1));

    // ROS自旋线程
    spin_thread = std::thread([this](){
        rclcpp::spin(node_);
    });

    // UI刷新定时器 30ms
    connect(&refresh_timer, &QTimer::timeout, this, &MainWindow::update_display);
    refresh_timer.start(30);

    // 下拉切换绑定
    connect(ui->combo_source, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::on_source_changed);
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown();
    if (spin_thread.joinable())
        spin_thread.join();
    delete ui;
}

void MainWindow::cb_shark(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat mat(msg->height, msg->width, CV_8UC3, msg->data.data());
    display_img = QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).rgbSwapped();
    current_source = SOURCE_SHARK_UDP;
}

void MainWindow::cb_custom(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat mat(msg->height, msg->width, CV_8UC3, msg->data.data());
    display_img = QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).rgbSwapped();
    current_source = SOURCE_CUSTOM_VIDEO;
}

void MainWindow::update_display()
{
    if (!display_img.isNull())
    {
        ui->label_video->setPixmap(QPixmap::fromImage(display_img.scaled(
            ui->label_video->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    }
}

void MainWindow::on_source_changed(int idx)
{
    switch(idx)
    {
        case 0: current_source = SOURCE_SHARK_UDP; break;
        case 1: current_source = SOURCE_CUSTOM_VIDEO; break;
    }
}