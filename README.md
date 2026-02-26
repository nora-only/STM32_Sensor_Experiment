# STM32 温湿度 & 心率传感器采集实验
数模混合电路设计实践课程实验

## 实验内容
- 使用 STM32 单片机采集心率传感器 / 温湿度传感器信号
- 通过 ADC 实现模拟信号采集
- 在定时器中断中处理数据、计算心率 BPM
- 数据实时显示在 OLED 屏

## 硬件
1. STM32F103C8T6开发板
2. PulseSensor 或其他光电式心率传感器模块
3. DHT11温湿度传感器模块
4. 0.96寸OLED显示屏（I2C接口）
5. 上拉电阻、杜邦线

## 工程结构
- Code/：完整 STM32 工程代码（已屏蔽自动生成的 HAL/CMSIS 驱动，体积更小）
  - 核心文件：`.ioc`（CubeMX配置）、`MDK-ARM/*.uvprojx`（Keil工程）
- Doc/：实验接线图、效果展示图

## 压缩包解压后运行指引
### 1. 环境要求
- Keil MDK-ARM V5.24 或更高版本
- STM32CubeMX V6.15 或更高版本（用于补全驱动文件）

### 2. 快速运行步骤
1. 将压缩包解压到无中文/空格的路径（Windows示例：`D:/STM32_Project/Sensor_Collect`；Linux/macOS示例：`~/STM32_Project/Sensor_Collect`）；
2. 打开STM32CubeMX软件，导入`Code/`目录下的`.ioc`配置文件；
3. 在 CubeMX 中直接点击「GENERATE CODE」按钮，自动补全 `Drivers/` 下的 CMSIS 和 HAL 驱动文件夹；
4. 用 Keil 打开 `Code/MDK-ARM/` 下的 `.uvprojx` 工程文件，点击「Build」编译后即可下载运行。

## ⚠️ 注意事项
1. 压缩包内未包含 HAL/CMSIS 驱动（体积大且可一键再生），需通过 CubeMX 补全，否则编译会报“驱动文件缺失”；
2. 若编译报错，优先检查解压路径是否含中文、CubeMX 生成驱动是否成功；
3. 清理编译产物可执行 Keil 菜单栏「Project → Clean Targets」，再删除 `MDK-ARM/` 下的 Listings/Objects 文件夹。

## 📌 核心代码说明
- 传感器数据采集：`Code/Core/Src/main.c`（ADC采集、定时器中断逻辑）
- 外设驱动：`Code/Drivers/BSP/`（OLED、DHT11、PulseSensor驱动）
- 心率BPM计算：`Code/Core/Src/tim.c`（定时器中断处理函数）
