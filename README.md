# 【RM2026】自定义客户端下部署模式低带宽落点图传 - 电科中山

用于RoboMaster部署模式下英雄机器人观测落点用的低带宽图传。

<img width="502" height="323" alt="视频封面" src="https://github.com/user-attachments/assets/a72e1683-be42-44d2-a3ae-7686517728fd" />

## 环境要求

- Ubuntu Linux
- ROS 2 Humble
- 海康/大恒相机 MVS SDK。

## 安装依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  python3-opencv python3-av \
  libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

安装ROS相关依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

本工程的相机图像采集代码由rm-vision项目修改而来；
`hik_camera` 依赖以下路径，确保路径里面文件都在就可以。

- 头文件：`/opt/MVS/include`
- 库文件：`/opt/MVS/lib/64`

## 编译启动
先`source`一下ROS的`setup.bash`。然后：
```bash
colcon build

source install/setup.bash
ros2 launch bringup sniper.launch.py
```
`sniper.launch.py`里面可以修改启动参数，比如图传分辨率，准星位置，dump图片用于调试，等等。详见文件内注释。

本工程仅为一个演示工程，开发过程中使用了LLM作为辅助。欢迎大家基于这个思路开发更好的自定义客户端。

## ubuntu配置ip
’’’bash
# 设置IPv4为手动模式，配置IP和子网掩码
nmcli connection modify "有线连接 1" ipv4.method manual ipv4.addresses 192.168.12.2/24
# 清空网关（直连不需要网关，避免路由冲突）
nmcli connection modify "有线连接 1" ipv4.gateway ""
# 关闭自动DNS
nmcli connection modify "有线连接 1" ipv4.ignore-auto-dns yes
# 重新激活连接，使配置立即生效
nmcli connection up "有线连接 1"
‘’’
'''
"https://subingwen.cn/cpp/protobuf/#1-1-%E6%BA%90%E7%A0%81%E5%AE%89%E8%A3%85"
'''

daheng sdk luj


- [daheng SDK](https://www.daheng-imaging.com/)
位置：下载中心 -> 软件下载 -> Galaxy_Linux_CN-EN_32bits/64bits
解压tar.gz后，用终端cd到文件夹路径，运行安装脚本 ./Galaxy_camera.run

sudo apt install ros-humble-serial-driver
sudo apt install ros-humble-asio-cmake-module
sudo apt install ros-humble-serial-driver ros-humble-io-context ros-humble-asio-cmake-module ros-humble-udp-driver
sudo apt install python3-paho-mqtt python3-protobuf