# 遥控器程序说明文档

遥控接收系统驱动程序。

DT7&DR16 2.4GHz遥控器采用D-BUS协议（普通UART取反）。
ET08&RF206S 2.4GHz遥控器采用S-BUS协议（普通UART取反）。

数据接收使用UART串口接收中断和空闲中断实现。

使用方法：

在hardware_config.h中配置遥控器接收机型号

```txt
connect_：用于监测遥控器连接状态

channel_：5个输入通道，包括左右摇杆横纵2个通道和1个拨轮，数值范围均为[-660，660]

switch_：2个拨杆输入，有 UP/MID/DOWN 3种状态

mouse_：鼠标移动信号x/y，鼠标按键状态press_l/r

key_：键盘按键

判断某个按键是否按下（以w键为例）：(rc.key_ & KEY_W)
```

相关资料：[RM遥控器套装](https://www.robomaster.com/zh-CN/products/components/detail/122)（注：遥控器用户手册中DBUS通信参数有误，单元数据长度为9）
