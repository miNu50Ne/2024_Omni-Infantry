# Alliance-EC 2024 Omni-Infantry

南京理工大学Alliance战队2024全向轮步兵下位机代码。

<img src=".assets/Alliance-badge.png" alt="Alliance-badge" style="zoom: 25%;" />




[TOC]




---



## 起源

这是湖南大学RoboMaster跃鹿战队电控组2022-2023赛季的通用嵌入式控制框架，可用于**机器人主控**、自研模组（imu/测距等各种传感器）、超级电容控制器等。

从目前的RoboMaster开源社区来看，大部分队伍都没有一套规则统一，符合较大规模软件开发原则的框架，有些学校连不同兵种代码都相去甚远，甚至连队伍用于传承的代码注释都寥寥无几，全靠师傅带徒弟言传身教。跃鹿框架作为一套模块化设计的优秀电控框架代码，被用于进行Alliance2024电控组的开发框架。



## 跃鹿框架设计思想

1. ***框架的设计模式***

   框架在结构上分为三层：bsp/module/app。整体使用的设计模式是**结构层级模式**，即每个“类”包含需要使用的底层“类”，通过组装不同的基础模实现更强大的功能。而最顶层的app之间则通过**pub-sub消息机制**进行解耦，使得在编写代码时不会出现相互包含的情况。

2. ***三层结构***

   - **bsp**即板级支持包，提供对开发板外设的软件抽象，让module层能使用和硬件无关的接口（由bsp提供）进行数据处理与交互。

     bsp层和ST的HAL为强耦合，与硬件直接绑定。若要向其他的ST芯片移植，基本不需要修改bsp层；若是其他单片机则建议保留**接口设计**，对接口调用进行重现实现。每一种外设的头文件中都定义了一个**XXXInstance**（xxx为外设名），其中包含了使用该外设所需要的所有数据，如发送/接收的数据，长度，id（如果有），父指针（指向module实例的指针，用于回调）等。由于C没有`class`，因此所有bsp的接口都需要传入一个额外的参数：XXXInstance*，用于实现c++的`this`指针以区分具体是哪一个实例调用了接口。

   - **module**即模块层，包括了需要开发板硬件外设支持的（一般用于通信）真实**硬件模组**如电机、舵机、imu、测距传感器，和通过软件实现的**算法**如PID、滤波器、状态观测器；还有用于兼容不同控制信息模块（遥控器/ps手柄/图传链路/上位机）的统一接口模块，以及为app层提供数据交互的message center。

   - **app**是框架层级中最高的部分。目前的框架设计里，会有多个app任务运行在freertos中，当然你也可以根据需要启动一些事件驱动的任务，所有的任务安排都放在`app/robot_task`中。当前的app层仅是一个机器人开发的示例，有了封装程度极高的module，你可以在app完成任何事情。

---

## 全向轮底盘解算

<img src="D:\alliance\electric control\2024_Omni-Infantry\.assets\output.png" style="zoom: 33%;" />

> 顺时针轮子序号：1，2，3，4 

### 参数定义

底盘坐标系——底盘自身坐标系

绝对坐标系——机器人运动方向坐标系

$v$——机器人速度矢量

$v_n$——对应轮子的径向速度

$v_x$——底盘坐标系下x轴速度

$v_y$——底盘坐标系下y轴速度

$w_z$——底盘坐标系和绝对坐标系下绕z轴转动

$v_{xcmd}$——cmd输入绝对坐标系下x轴速度

$v_{ycmd}$——cmd输入绝对坐标系下y轴速度

$R$——机器人轴心至轮子距离

$\theta$——底盘坐标系与绝对坐标系的角度误差

坐标系采用右手系：

<img src="D:\alliance\electric control\2024_Omni-Infantry\.assets\ML-4.png" style="zoom:67%;" />

### 逆运动学解算

规定当机器人底盘正方向为y轴正方向。即底盘朝向北时，x轴正方向为东。

底盘在绝对坐标系下沿x轴和y轴平动，绕z轴转动

机器人底盘速度矢量沿x和y轴分解：
$$
\vec{v}=\vec{v_x}+\vec{v_y}+\vec{w_z}*R
$$

#### 底盘坐标系与绝对坐标系重合：

$$
v_1=v_x+v_y+w_z*R\\
v_2=v_x+-v_y+w_z*R\\
v_3=-v_x+-v_y+w_z*R\\
v_4=-v_x+v_y+w_z*R\\
$$

#### 底盘坐标系与绝对坐标系有角度误差：

以逆时针为正方向：
$$
v_x=v_{xcmd}*cos\theta-v_{ycmd}*sin\theta\\
v_y=v_{xcmd}*sin\theta+v_{ycmd}*cos\theta
$$
代入原方程组：
$$
v_1=v_{xcmd}*cos\theta-v_{ycmd}*sin\theta+v_y+w_z*R\\
v_2=v_{xcmd}*cos\theta-v_{ycmd}*sin\theta+-v_y+w_z*R\\
v_3=-v_{xcmd}*cos\theta-v_{ycmd}*sin\theta+-v_y+w_z*R\\
v_4=-v_{xcmd}*cos\theta-v_{ycmd}*sin\theta+v_y+w_z*R\\
$$