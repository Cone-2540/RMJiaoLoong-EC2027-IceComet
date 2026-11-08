# 双关节机械臂中期项目

## 工程内容

| 目录 | 内容 |
|---|---|
| `app` | 用户层代码 |
| `base` | 电机、遥控器、PID、滤波等基础模块 |
| `interface` | 任务调度、回调和机械臂实例 |
| `mcu` | STM32 工程、HAL 和 FreeRTOS |
| `lib` | 数学库 |
| `web` | 任意轨迹网页编辑器、本地上传服务 |

任意轨迹入口：[轨迹画板](http://127.0.0.1:8002/)，操作与代码位置见 [任意轨迹绘制](doc/任意轨迹绘制.md)。

完成后的控制调用关系：

```text
controlTask（1 ms）
  rc.handle()
  controlLoop()
    robotControl()
    arm.handle()
      updateFeedback()
      trajectoryPlanner()
      writeMotorCommands()
  allMotorsHandle()

canTask
  canTxMonitor()
```
