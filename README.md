<div style="text-align:center;">
    <a href="https://github.com/ittuann/RoboMaster_UAV-Gimbal_Ares2022">
        <img src=".\Pic\Banner.png" alt="Banner" width="80%">
    </a>
</div>

第二十一届 RoboMaster 全国大学生机器人大赛 机甲大师赛 Ares战队 空中机器人六轴无人机云台程序框架开源

项目希望探索出，能够给 RoboMaster 比赛中的所有机器人都能使用的一套代码架构，即不同的机器人仅需要修改程序中的结构体和控制器即可复用。因此在设计程序架构时，尽量提高了代码的复用性，并将函数模块化和参数化。

开发环境 STM32CubeIDE - V1.9.0, STM32Cube FW_F4 Package - 1.27.0, FreeRTOS Version - 10.3.1, CMSIS-RTOS Version - 1.02, DSP Library - 1.2.1

单片机为RM开发板C型，STM32F407IGHx

开源了程序框架用于讨论。 ~~赛后会整理完整的项目工程~~  无人机摔了三次，项目被迫暂停，已上传目前的完整框架工程。

RoboMaster论坛链接: <https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22085>

------

# 代码框架图

<img src=".\Pic\CodeArchitecture.png" alt="CodeArchitecture" width="50%">

节点机制和消息收发机制是这套框架的亮点。实现了很大程度的代码复用和模块化，以及上层逻辑代码与底层硬件配置代码的隔离。

- HAL层

  HAL层是对硬件接口的封装抽象。包含了操作系统的线程设置，还有CubeMX使用HAL库对部分外设配置的初始化，同时也包含对HAL库的重封装。

- Hardware层

  Hardware层的设置，是为了将底层硬件的配置代码与上层的逻辑代码分离。包含了一些外设的必要驱动，通讯协议以及解包函数，以及设备状态监控。

- Message层

  Message层作为消息框架，是一个消息发布和订阅的机制。用于各个模块之间的消息传递，例如APP与APP之间、APP与Driver之间的通讯。同时适用于管理一对一和一对多的消息收发情况。

- Nodes层

  Nodes层存储通用的算法函数库，如PID，Kalamn，Filter等。使用结构体封装或通过函数指针等方式实现，能很大程度简化结构，提高程序通用性和代码模块复用度，也能够拥有不错的灵活性。

- Application层

  Application层为各种逻辑代码，如云台、底盘、发射机构等模块的控制任务。
