# 三段式爬楼机器人 – F 模块（通用支撑层）

## 模块概述

F 模块是整个三段式爬楼机器人项目的**通用支撑层**，提供跨模块共享的数据类型、全局配置、工具函数以及日志记录功能。本层为所有上层模块（传感器、执行器、状态机等）提供统一的基础设施，确保代码的可移植性和可维护性。

---

## 文件结构
include/
├── config.h # 全局硬件引脚、传感器阈值、运动参数等配置
├── types.h # 通用数据类型、枚举、结构体定义
├── utils.h # 工具函数声明（限幅、滤波、超时检测）
├── logger.h # 日志接口声明
src/
├── utils.cpp # 工具函数实现
└── logger.cpp # 日志实现（带时间戳和等级输出）

text

---

## 文件说明

### `config.h`
存放所有与硬件相关的常量配置，包括：
- I2C 总线、PCA9685 地址、PWM 频率
- 电机驱动、舵机、线性执行器的引脚映射
- 传感器（超声波、下视、IMU）的阈值与超时参数
- 几何尺寸、运动速度、安全限制等

**重要常量示例**：
```cpp
namespace RobotConfig {
    namespace GPIO {
        constexpr int FRONT_L_IN1 = 17;   // 前左电机方向引脚
    }
    namespace Motion {
        constexpr float APPROACH_SPEED = 0.35f;
    }
    namespace Safety {
        constexpr float MAX_SAFE_PITCH_DEG = 25.0f;
    }
}
types.h
定义整个项目通用的数据类型和枚举：

MotionState：机器人状态机状态（空闲、接近台阶、前段爬升等）

FaultCode：故障码枚举

结构体：PoseData（姿态）、DistanceReading（距离）、DownwardReading（下视）、AxisState（直线轴状态）等

回调函数类型：PoseCallback、DistanceCallback 等

utils.h / utils.cpp
提供通用工具函数：

clamp()：数值限幅

lowPassFilter()：一阶低通滤波

hasTimedOut() / isFresh()：数据新鲜度判断

logger.h / logger.cpp
简单的日志系统，支持四个等级：

Logger::debug() 调试信息

Logger::info() 常规信息

Logger::warn() 警告

Logger::error() 错误

日志输出格式：[HH:MM:SS] [LEVEL] message

使用方法
1. 包含头文件
在需要使用配置、类型或工具的源文件中包含相应头文件：

cpp
#include "config.h"
#include "types.h"
#include "utils.h"
#include "logger.h"
2. 使用日志
cpp
Logger::info("Robot initializing...");
if (error_occurred) {
    Logger::error("Failed to start IMU sensor");
}
3. 使用工具函数
cpp
float limited_speed = Robot::clamp(speed, -1.0f, 1.0f);
bool fresh = Robot::isFresh(last_timestamp, std::chrono::milliseconds(100));