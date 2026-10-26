# 电控-视觉通信程序说明文档

电控-视觉通信（主控板-minipc通信），用于自瞄、能量机关、导航等由视觉辅助实现的功能的通信数据收发。通信具体数据内容在[cv_protocol.h](../base/cv_comm//cv_protocol.h)中定义（该文件由电控部和视觉部共同维护）。

## 数据帧格式

```txt
串口配置：
波特率115200，8位数据位，1位停止位，无硬件流控，无校验位

数据帧格式：
header(3-byte) + data(n-byte) + tail(2-byte)

帧头段header：1-byte起始位SOF(0x23) + 1-byte包ID + 1-byte数据段字节数data_len(n)
数据段data：n-byte数据
帧尾CRC校验：2-byte CRC16校验
```

## 通信规则

### 发送

不同工作模式（CVMode_e mode_）下，发送general数据包和该模式下需要的功能数据包，例如自瞄模式发送general和aimshoot数据包，导航模式发送general和navigation数据包。视觉程序根据收到的general数据包内容开关功能线程。不同数据包的发送频率在 **CVComm::txMonitor()** 中调整。

各应用程序通过向CVComm类对象的对应数据包写值来设置发送数据包中的具体内容。

### 接收

所有工作模式下均可接收通信协议中定义的任何数据包，成功接收到的数据包储存在CVComm的对象中，各应用程序从CVComm中读取需要的数据。
