# 基于坐标的机器人运动包
## 全局坐标移动
- 发布名为world_locate的话题，消息内容为geometry_msgs::Twist，包含了平面坐标系下的二维坐标和垂直地面Z轴的角度
## 自转及公转
- 发布名为rotate_cmd的话题，消息内容为geometry_msgs::Twist，偷了个懒，用linear的x,y,z表示公转的圆心x,y及速度，同理用angular表示自转。

