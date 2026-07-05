#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), current_source_(SOURCE_MQTT_LOCAL)
{
    ui->setupUi(this);

    // ========== 对齐官方规范：窗口尺寸配置 ==========
    // 初始窗口大小：1280x720，16:9标准比例，与官方选手端一致
    this->resize(1280, 720);
    // 最小窗口尺寸：960x540，防止过度缩放导致界面元素错乱
    this->setMinimumSize(960, 540);
    // 窗口标题对齐官方风格
    this->setWindowTitle("RoboMaster 自定义客户端");
    // ==================================================

    node_ = rclcpp::Node::make_shared("qt_video_gui");

    // 第1路：MQTT CustomByteBlock 解码画面 → /decoded_image/local
    sub_mqtt_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/decoded_image/local",
        rclcpp::QoS(10).best_effort(),
        std::bind(&MainWindow::callbackMqttImg, this, std::placeholders::_1)
    );

    // 第2路：UDP Shark 远程原生图传 → /decoded_image/shark
    sub_shark_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/decoded_image/shark",
        rclcpp::QoS(10).best_effort(),
        std::bind(&MainWindow::callbackSharkImg, this, std::placeholders::_1)
    );

    // 第3路：本地大恒相机原生画面 → /image_raw
    sub_local_cam_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::QoS(1).best_effort(),
        std::bind(&MainWindow::callbackLocalCam, this, std::placeholders::_1)
    );

    // 裁判系统状态数据
    sub_game_status_ = node_->create_subscription<std_msgs::msg::String>(
        "/referee/game_status",
        10,
        std::bind(&MainWindow::callbackRefereeStatus, this, std::placeholders::_1)
    );

    connect(ui->combo_video_source, SIGNAL(currentIndexChanged(int)),
            this, SLOT(slotSwitchVideoSource(int)));

    refresh_timer_.setInterval(30);
    connect(&refresh_timer_, &QTimer::timeout, this, &MainWindow::slotUpdateDisplay);
    refresh_timer_.start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

// ====================== 三路图像回调：统一BGR格式输出 ======================

// 本地相机原生画面：修复偏色bug，cv_bridge已输出BGR8，无需二次转换
void MainWindow::callbackLocalCam(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        mat_local_cam_ = cv_ptr->image.clone();
    }
    catch(cv_bridge::Exception &e)
    {
        RCLCPP_WARN(node_->get_logger(), "Local camera image convert err: %s", e.what());
    }
}

// Shark UDP 解码画面：BGR8直接深拷贝
void MainWindow::callbackSharkImg(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        mat_shark_ = cv_ptr->image.clone();
    }catch(cv_bridge::Exception &e){
        RCLCPP_WARN(node_->get_logger(), "Shark image convert err: %s", e.what());
    }
}

// MQTT CustomByteBlock 解码画面：BGR8直接深拷贝
void MainWindow::callbackMqttImg(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        mat_mqtt_ = cv_ptr->image.clone();
    }catch(cv_bridge::Exception &e){
        RCLCPP_WARN(node_->get_logger(), "MQTT image convert err: %s", e.what());
    }
}

void MainWindow::callbackRefereeStatus(const std_msgs::msg::String::SharedPtr msg)
{
    referee_text_ = QString::fromStdString(msg->data);
}

void MainWindow::slotSwitchVideoSource(int idx)
{
    current_source_ = static_cast<VideoSource>(idx);
    ui->label_video->clear();
}

// ====================== 十字准星绘制（BGR格式下生效） ======================
void MainWindow::drawCrosshair(cv::Mat &img)
{
    if(img.empty()) return;

    int h = img.rows;
    int w = img.cols;
    int cx = w / 2;
    int cy = h / 2;

    // 十字线（淡紫色，与解码器样式一致）
    cv::Scalar cross_color = cv::Scalar(235, 190, 230);
    int line_width = 1;
    cv::line(img, cv::Point(0, cy), cv::Point(w - 1, cy), cross_color, line_width, cv::LINE_AA);
    cv::line(img, cv::Point(cx, 0), cv::Point(cx, h - 1), cross_color, line_width, cv::LINE_AA);

    // 中心圆圈（淡绿色）
    cv::Scalar center_color = cv::Scalar(170, 255, 170);
    cv::circle(img, cv::Point(cx, cy), 24, center_color, 1, cv::LINE_AA);
}

// ====================== 画面渲染主逻辑 ======================
void MainWindow::slotUpdateDisplay()
{
    rclcpp::spin_some(node_);
    cv::Mat show_mat;

    switch(current_source_)
    {
        case SOURCE_MQTT_LOCAL:
            if(!mat_mqtt_.empty()) show_mat = mat_mqtt_;
            break;
        case SOURCE_SHARK_UDP:
            if(!mat_shark_.empty()) show_mat = mat_shark_;
            break;
        case SOURCE_LOCAL_CAM_RAW:
            if(!mat_local_cam_.empty()) show_mat = mat_local_cam_;
            break;
        default:
            break;
    }

    if(!show_mat.empty())
    {
        // 绘制十字准星（BGR格式下绘制，颜色正确）
        drawCrosshair(show_mat);

        // BGR转RGB，适配Qt显示格式
        cv::Mat rgb_mat;
        cv::cvtColor(show_mat, rgb_mat, cv::COLOR_BGR2RGB);

        QImage img(
            rgb_mat.data,
            rgb_mat.cols,
            rgb_mat.rows,
            static_cast<int>(rgb_mat.step),
            QImage::Format_RGB888
        );

        // 保持宽高比平滑缩放，居中显示（与官方显示逻辑一致）
        ui->label_video->setPixmap(
            QPixmap::fromImage(img.copy()).scaled(
                ui->label_video->size(),
                Qt::KeepAspectRatio,
                Qt::SmoothTransformation
            )
        );
    }
    else
    {
        ui->label_video->clear();
    }

    ui->text_referee_data->setPlainText(referee_text_);
}
