# 流式视频数据包 - 固定300字节
uint64 sequence_id      # 全局包序号
uint64 timestamp_ns     # ROS2 时间戳
uint8[300] data         # 固定300字节负载（最后一包不足则补零）