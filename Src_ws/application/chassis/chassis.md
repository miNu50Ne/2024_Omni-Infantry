# chassis
## 工作流程

首先进行初始化，`ChasissInit()`会被`RobotInit()`调用，进行裁判系统、底盘电机的初始化。如果为双板模式，则还会初始化IMU，并且将消息订阅者和发布者的初始化改为`CANComm`的初始化。

操作系统启动后，工作顺序为：

1. 从cmd模块获取数据（如果双板则从CANComm获取）
2. 判断当前控制数据的模式，如果为停止则停止所有电机
3. 根据控制数据，计算底盘的旋转速度
4. 根据控制数据中yaw电机的编码器值`angle_offset`，将控制数据映射到底盘坐标系下
5. 进行麦克纳姆轮的运动学解算，得到每个电机的设定值
6. 获取裁判系统的数据，并根据底盘功率限制对输出进行限幅
7. 由电机的反馈数据和IMU（如果有），计算底盘当前的真实运动速度
8. 设置底盘反馈数据，包括运动速度和裁判系统数据
9. 将反馈数据推送到消息中心（如果双板则通过CANComm发送）

## 全向轮底盘解算

<img src="D:\alliance\electric control\2024_Omni-Infantry\.assets\output.png" style="zoom: 33%;" />

> 顺时针轮子序号：1，2，3，4 

### 参数名词解释

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