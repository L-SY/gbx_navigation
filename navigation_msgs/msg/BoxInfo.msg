# BoxInfo.msg
string box_id             # 箱子的RFID标识符
string box_type           # 箱子类型
time enter_time          # 箱子进入时间
int32 rssi               # RFID信号强度
string status            # 箱子状态（比如 "in_cabinet", "removed", "expired" 等）
string extra_info        # 预留的额外信息字段，可以是JSON字符串