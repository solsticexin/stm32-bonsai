# STM32 智能盆栽监控系统

这是一个基于 STM32F103C8T6 微控制器的智能盆栽监控系统，使用 Rust 编程语言开发。该系统能够实时监测盆栽的环境参数，并通过串口与上位机通信，实现远程控制和数据采集。

## 项目概述

本项目实现了一个完整的嵌入式系统，包括：

- **传感器数据采集**：温度、湿度、土壤湿度、光照强度
- **执行器控制**：水泵、补光灯、风扇、蜂鸣器
- **通信协议**：与 ESP32 等上位机通过串口通信
- **实时控制**：支持脉冲控制和状态切换

## 硬件配置

- **MCU**: STM32F103C8T6 (Blue Pill)
- **传感器**:
  - DHT11: 温度和湿度传感器
  - BH1750: 光照强度传感器 (I2C 接口)
  - 土壤湿度传感器: 模拟量输入 (ADC)
- **执行器**:
  - 水泵 (PB0)
  - 补光灯 (PB1)
  - 风扇 (PB10)
  - 蜂鸣器 (PA1，低电平触发)
- **通信**:
  - USART1: PA9(TX), PA10(RX), 115200bps

## 嵌入式知识介绍

### ADC 读取

ADC (Analog-to-Digital Converter) 用于将模拟信号转换为数字信号。在本项目中，土壤湿度传感器输出模拟电压，通过 ADC 转换为数字值。

**原理**：
- STM32F103 的 ADC 分辨率为 12 位 (0-4095)
- 采样时间可配置以平衡速度和精度
- 土壤传感器在干燥时电阻大 (电压高)，湿润时电阻小 (电压低)
- 转换公式：百分比 = 100 - (ADC值 * 100 / 4095)

**代码实现**：
```rust
pub fn read_percent(adc: &mut Adc<ADC1>, pin: &mut PA0<Analog>) -> u8 {
    adc.set_sample_time(SampleTime::T_239);
    let raw: u16 = block!(adc.read(pin)).unwrap_or(0u16);
    let percent = if raw >= 4095 {
        0
    } else {
        100_u32.saturating_sub((raw as u32 * 100) / 4095).min(100) as u8
    };
    percent
}
```

### DHT11 读取

DHT11 是数字温湿度传感器，使用单总线协议通信。

**原理**：
- 单总线通信协议：主机发送启动信号，传感器响应并发送数据
- 数据格式：40位 (8位湿度整数 + 8位湿度小数 + 8位温度整数 + 8位温度小数 + 8位校验和)
- 时序要求严格：需要精确的微秒级延时
- 校验和：前4字节相加应等于第5字节

**通信时序**：
1. 主机拉低18ms后释放
2. 传感器响应：80us低电平 + 80us高电平
3. 数据传输：每个位以50us低电平开始，高电平持续时间决定0/1 (26-28us=0, 70us=1)

### I2C 读取

I2C (Inter-Integrated Circuit) 是一种串行通信协议，用于连接多个设备。

**原理**：
- 双线制：SDA (数据线) + SCL (时钟线)
- 主从架构：STM32 作为主机，BH1750 作为从机
- 地址：BH1750 默认地址 0x23
- 通信流程：
  1. 起始条件
  2. 发送从机地址 + 写命令
  3. 发送命令字节
  4. 停止条件
  5. 延时等待转换
  6. 起始条件
  7. 发送从机地址 + 读命令
  8. 读取数据字节
  9. 停止条件

**BH1750 光照传感器**：
- 命令：0x20 (一次性高分辨率模式)
- 输出：16位数据，需要除以1.2转换为Lux值

### 串口读取

USART (Universal Synchronous/Asynchronous Receiver/Transmitter) 用于与上位机通信。

**配置**：
- 波特率：115200 bps
- 数据位：8位
- 停止位：1位
- 校验：无

**接收处理**：
- 中断驱动：每接收一个字节触发中断
- 行缓冲：累积接收直到换行符
- 协议解析：自定义命令格式 (target:action:time)

## 快速开始

请参考 [getting-started.md](getting-started.md) 获取详细的构建和烧录指南。

## 项目结构

```
src/
├── main.rs          # 主程序入口，RTIC 任务定义
├── actuators.rs     # 执行器控制逻辑
├── protocol.rs      # 通信协议实现
└── sensors/         # 传感器模块
    ├── mod.rs       # 传感器接口定义
    ├── dht11.rs     # DHT11 温湿度传感器
    ├── soil.rs      # 土壤湿度传感器
    └── bh1750.rs    # BH1750 光照传感器
```

## 通信协议

系统使用自定义文本协议通过串口通信：

- **命令格式**: `target:action:time\n`
- **响应格式**: `ack:target:action:result:message\n`
- **数据上报**: `data:temperature:humidity:soil:lux:buzzer:water:light:fan\n`

## 许可证

本项目采用双重许可证：Apache-2.0 和 MIT。

## 贡献

欢迎提交 Issue 和 Pull Request！

## 行为准则

本项目遵循 [Rust Code of Conduct](https://www.rust-lang.org/policies/code-of-conduct)。
