# Power_Control功率控制模块

## todo

1.目前的电机功率模型中的参数是直接使用的西交利物浦在开源之中给出的参数，后续将会开始着手建立自己的电机模型

2.完成对除M3508以外电机的支持

##快速使用教学

1.初始化：在chassis.c中的ChassisInit()函数里设置自己所写机器人轮电机的减速比
    //修改减速比以及最大功率
    PowerControlInstance power_init = {
        .coefficient.reduction_ratio = 0.0520746310219994f};
    power = PowerControlInit(&power_init);
2.使用：在LimitChassisOutput()中对四个电机分别进行限幅计算。
       例：lf_limit = PowerControlCalc(power, motor_lf->measure.real_current, motor_lf->measure.speed_aps);
       随后将得到的值作为参考值输入DJIMotorSetRef();并在底盘任务中调用LimitChassisOutput()
       出于解耦的目的，请自行调用裁判系统给出的最大功率并保证每次执行底盘任务的时候该限制都可以被重新赋值。

##功率控制介绍

鸽了，代码好用了再写