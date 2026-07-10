# 【RM2026】自定义客户端 - 电科中山

## 环境要求

- Ubuntu22.04 Linux
- ROS 2 Humble
```bash
wget http://fishros.com/install -O fishros && . fishros
```
- 海康/大恒相机 MVS SDK。
- [daheng SDK](https://www.daheng-imaging.com/)
位置：下载中心 -> 软件下载 -> Galaxy_Linux_CN-EN_32bits/64bits
解压后，用终端cd到文件夹路径，运行安装脚本 ./Galaxy_camera.run

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
  gstreamer1.0-plugins-ugly gstreamer1.0-libav \
  ros-humble-serial-driver ros-humble-asio-cmake-module \
  ros-humble-serial-driver ros-humble-io-context ros-humble-asio-cmake-module ros-humble-udp-driver \
  python3-paho-mqtt python3-protobuf
```
- [protobuf](https://subingwen.cn/cpp/protobuf/#1-1-%E6%BA%90%E7%A0%81%E5%AE%89%E8%A3%85)

安装ROS相关依赖,在此路径（/home/rm/Pacific_doorlock_sniper-main）下运行：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## ubuntu配置ip

### 设置IPv4为手动模式，配置IP和子网掩码
```bash
nmcli connection modify "有线连接 1" ipv4.method manual ipv4.addresses 192.168.12.2/24
```
### 清空网关（直连不需要网关，避免路由冲突）
```bash
nmcli connection modify "有线连接 1" ipv4.gateway ""
```
### 关闭自动DNS
```bash
nmcli connection modify "有线连接 1" ipv4.ignore-auto-dns yes
```
### 重新激活连接，使配置立即生效
```bash
nmcli connection up "有线连接 1"
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




