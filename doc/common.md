# common-常用函数/通用功能包/算法实现

## math

常用数学工具，如限幅，循环限幅，单位转换函数

## connect

连接状态检测，一般用于检测通信状态和通信频率。

使用方法:

```c++
Connect::Connect(uint32_t timeout)
构造函数，设置连接超时时长（单位ms） 

bool Connect::check()
运行过程中调用，更新并返回连接状态。

ConnectEdge_e Connect::refresh()
接收到信息时调用，返回连接状态变化情况（不变 / 未连接<->连接）

另外可通过Connect::freq()和Connect::lastTick()获取频率和时间信息。
```

## PID

PID控制算法实现

使用方法：

```c++
PID::PID(float kp, float ki, float kd, float i_max, float out_max)
构造函数，设置PID参数

float PID::calc(float ref, float fdb)
PID计算，输入：ref-参考值，fdb-反馈值
```

## filter

低通滤波

使用方法：

```c++
LowPassFilter::LowPassFilter(float k, float init)
设置滤波参数k和初值

float LowPassFilter::update(float input)
输入原始信号，返回滤波后信号

其他函数自行阅读代码
```

## Kalman Filter

卡尔曼滤波实现。

使用方法：

```c++
KalmanFilter::KalmanFilter(uint16_t x_size, uint16_t u_size, uint16_t z_size, const float* F, const float* B, const float* H, const float* Q, const float* R, const float* x0)
构造函数输入x, u, z向量的维数和卡尔曼滤波各矩阵，或不输入维数默认为1维卡尔曼滤波

void KalmanFilter::update(float* x, float* u, float* z, uint16_t x_size, uint16_t u_size, uint16_t z_size)
更新时调用update函数或updateOffline函数，输入储存x, u, z向量的数组/指针，根据输入u和输出z更新状态x
```

相关资料：[资料1](https://zhuanlan.zhihu.com/p/39912633)，[资料2（共4篇）](https://zhuanlan.zhihu.com/p/338269917)

## CRC

CRC8，CRC16校验和添加校验码，在视觉通信/裁判系统通信中使用

## fifo_buffer

简易fifo实现，用于实现通信数据先入先出（first in first out），降低丢包概率。

使用方法

```c++
FIFOBuffer::FIFOBuffer(uint8_t* pdata, uint32_t max_size)
构造函数，设置数据储存数组地址和缓冲区容量（数组大小）

void FIFOBuffer::append(uint8_t* data, uint32_t size)
接收到数据时调用，添加数据到缓冲区，超出缓冲区容量部分自动按FIFO形式移除

void FIFOBuffer::remove(uint32_t size)
确定缓冲区中需要移除的数据长度时（一般为完整正确接收一个数据包时）调用

void FIFOBuffer::clear(void)
需要清空缓冲区或不确定缓冲区中需要移除的数据长度时（如通信校验失败）调用，直接清空缓冲区数据

uint32_t FIFOBuffer::find(uint8_t data)
查找数据，返回首个结果的序号，未找到则返回size
```

## bool_input

布尔输入类，可用于按键、拨杆、鼠标或其他bool状态的检测、边沿触发处理，以及实现长按后类似id设置的功能。

使用方法：

```c++
BoolInput::BoolInput(uint32_t timeout = UINT32_MAX, uint32_t t_input = 10)
构造函数，设置长按判断阈值时长（timeout, ms）和消抖判断时长（t_input, ms）

void BoolInput::handle(bool input_source)
在主函数内调用，输入参数可以是按键电平/遥控器拨杆/鼠标按键等判断条件。函数内处理输入状态并调用回调函数。
```

BoolInput类通过定义回调函数的方式定义输入状态的响应

```c++
void (*inputCallback)(bool); 输入状态回调
void (*inputEdgeCallback)(bool); 输入边沿回调
void (*cmdInputCallback)(bool); 指令内输入状态回调
void (*cmdStartCallback)(void); 判定长按/指令开始回调
void (*cmdFinishCallback)(uint8_t); 长按/指令结束回调
```

示例（实现类似于C610电调按键设置id的功能）

```c++
BoolInput key(1500);

void keyInputCallback(bool flag) {
  if (flag) {
    led.setColor(255, 0, 0);
  } else {
    led.setColor(0, 0, 255);
  }
}

void keyCmdStartCallback(void) {
  led.setColor(150, 150, 0);
}

void keyCmdFinishCallback(uint8_t cmd) {
  led.setColor(0, 0, 255);
  led.setModeBlink(cmd);
}

void keyCmdInputCallback(bool flag) {
  if (flag) {
    led.setModeOn();
  } else {
    led.setModeOff();
  }
}

key.inputCallback = keyInputCallback;
key.cmdStartCallback = keyCmdStartCallback;
key.cmdFinishCallback = keyCmdFinishCallback;
key.cmdInputCallback = keyCmdInputCallback;
```

## matrix

CMSIS DSP库函数封装，该库当前版本求逆函数测试有bug，已另行实现。
