# 变长视频数据包 - 专用于UDP高码率链路，无固定长度填充
uint32 sequence_id
uint64 timestamp_ns
uint8[] data
