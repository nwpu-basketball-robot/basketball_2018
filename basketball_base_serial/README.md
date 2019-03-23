# 机器人串口通信模块
## 功能介绍
1. 接收下位机传来的信息，根据func字段的不同来发布不同的话题，话题名为:RecvData + func, 消息类型为robot_state：int32的func及一个vector<double>
2. 接收其他的机器人信息消息，并通过串口发送给下位机，接收的消息类型为robot_message：vector<uint8_t>

## Tip
- func数值及对应功能参考“篮球机器人底盘通信协议”，其中也规定了数据帧的内容和结构。
