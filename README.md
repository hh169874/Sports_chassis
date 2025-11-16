# 运动底盘
## 一、项目简介
本项目包含：全向轮/麦轮运动底盘的运动解算，大疆M2006电机驱动，双环pid，流程控制执行框架等相关代码。适用于工创赛、电赛、智能车、起重机等赛事。项目基于stm32f407vet6开发，使用clion+STM32CubeMX作为开发开发工具进行开发，使用freertos框架。  

⛔ 本项目使用freertos框架，存在栈溢出风险，切勿直接照搬整个项目参加比赛。  

✅ 推荐移植相关功能模块。  
  
## 二、项目背景
我作为电控参加2025年中国大学生机械工程创新创意大赛物流技术创意赛（起重机），负责运动控制部分，现在比赛结束开放源码。在开发阶段，我发现关于摩擦轮的完整代码比较少，基本上都是理论讲解，配合一些代码，并没有特别完整的代码实例，所以当时编写和验证运动解算代的码花了很长时间。我项目中的包含了比较完整的示例代码，同时再md中也会进行一些讲解，希望对大家有所帮助。  
  
## 三、项目方案
<img width="1230" height="699" alt="image" src="https://github.com/user-attachments/assets/f5cb7b11-54ae-4d18-b66b-8663a85e58e3" />
<img width="1259" height="616" alt="image" src="https://github.com/user-attachments/assets/413de636-5c8e-4801-892b-454a961a3bfa" />
  
## 四、项目结构
由于比赛周期比较紧，加之我比较懒，所以这个项目并没有分层，驱动和应用基本都放在Core/Src/bsp这个文件夹中，但是不用担心，下面会详细介绍这些文件是干什么用的。
- src
  - bsp
    - Arm_Control.c 机械臂控制，货盘控制代码
    - Arm_Control.h
    - Chassis_Control.c 全向轮/麦轮正逆解算（包含坐标系变换）
    - Chassis_Control.h
    - Gyro.c 串口陀螺仪控制
    - Gyro.h
    - HC_SR04.c 超声波距离传感器驱动（已弃用）
    - HC_SR04.h
    - MPU9250.c MPU9250陀螺仪驱动（已弃用）
    - MPU9250.h
    - OLED.c 0.96寸OLED显示屏驱动程序
    - OLED.h
    - OLED_Data.c 0.96寸OLED显示屏字模数据
    - OLED_Data.h
    - Process_Control.c 流程控制
    - Process_Control.h
    - VL53L1 tof激光测距仪库文件（已弃用）
      - core
      - platform
    - VL53L1.c tof激光测距仪驱动
    - VL53L1.h
    - bsp_can.c 大疆m2006电机驱动（发送命令/接收解析）
    - bsp_can.h
    - eMPL MPU9250陀螺仪库文件（已弃用）
    - map.c 全局坐标系下当前小车位置存储，地图
    - map.h
    - mpu6050.c mpu6050驱动
    - mpu6050.h
    - pid.c 双环pid实现
    - pid.h
    - ps2.c ps2手柄控制驱动，用于pid调参
    - ps2.h
  - can.c 以下为cubemx自动生成文件
  - dma.c
  - freertos.c freertos任务定义
  - gpio.c
  - i2c.c
  - main.c
  - spi.c
  - stm32f4xx_hal_msp.c
  - stm32f4xx_hal_timebase_tim.c
  - stm32f4xx_it.c
  - syscalls.c
  - sysmem.c
  - system_stm32f4xx.c
  - tim.c
  - usart.c
## 五、项目主要内容详解
📌 项目中均有比较完整的代码和注释，配合项目代码理解更佳  
<br>
### 1、全向轮/麦轮的控制及定位（Chassis_Control.c/h）
首先我们需要了解一下上面是运动正解，什么是运动逆解。下面用一些比较通俗的话来解释。  
- 运动逆解：已知整体位姿，反求各个关节/电机的角度。简单来说就是输入的是小车的速度/位置，输出的是麦轮/全向轮四个电机的转速/转动距离（这里的速度和位置均为三轴数据x、y、z（旋转））。
- 运动正解：已知各个关节/电机的角度，求整体位姿。简单来说输入的是四个电机的转速/转动距离，输出的是小车的速度/位置。
  
下面说明一下在项目中的具体应用（涉及双环pid部分）  
- 运动逆解：在速度pid中，我如何设定小车的速度呢，比如我想让小车沿x方向以1m/s的速度运动时，应该输入（1,0,0），那么就会解算得到四个电机的线速度，用轮子半径将线速度转换到角速度，再通过速度pid计算电流值，然后将电流值发送给电机，即可完成控制。
- 运动正解：在位置pid中，我如何知道现在小车运行到了什么位置呢？那就可以用运动正解，通过CAN电机报文反馈，我可以拿到四个电机转动的总角度，用这个角度和轮子周长计算可以得到四个轮子运动的距离，将这个四个距离输入到运动正解后，我们就可以得到小车的当前的位置（x,y,z）。
  
既然我们已经知道了运动正解和运动逆解是干什么的了，那么接下来就简单了，根据你使用的摩擦轮类型和车体结构构造出你需要的正解/逆解公式即可，这里我就不详细讲了，但是我会给一些比较有用的文章链接，不需要去分析什么摩擦力那些的，直接把他公式抄下来就行了。  
- 这个教程比较完整，且有三视图，便于理解。但是麦轮对侧的方向他并没有进行反向操作[底盘逆运动学解算](https://zju-helloworld.github.io/Wiki/%E7%BB%84%E4%BB%B6%E8%AF%B4%E6%98%8E%EF%BC%88%E6%97%A7%EF%BC%89/%E6%9C%BA%E5%99%A8%E4%BA%BA%E9%80%9A%E7%94%A8%E7%BB%84%E4%BB%B6/%E7%AE%97%E6%B3%95/%E5%BA%95%E7%9B%98%E9%80%86%E8%BF%90%E5%8A%A8%E5%AD%A6%E8%A7%A3%E7%AE%97/)  
- 这个教程包含正解和逆解 [ROS机器人学习——麦克纳姆轮运动学解算-CSDN博客](https://blog.csdn.net/oxiaolingtong/article/details/120198677?ops_request_misc=elastic_search_misc&request_id=f61b05590901f51c8a76eae319320e9f&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-120198677-null-null.142^v102^pc_search_result_base5&utm_term=ROS%E6%9C%BA%E5%99%A8%E4%BA%BA%E5%AD%A6%E4%B9%A0%E2%80%94%E2%80%94%E9%BA%A6%E5%85%8B%E7%BA%B3%E5%A7%86%E8%BD%AE%E8%BF%90%E5%8A%A8%E5%AD%A6%E8%A7%A3%E7%AE%97&spm=1018.2226.3001.4187)  
这两个文章我会也会打包发在项目中。
<br>⚠️注意事项：注意对侧电机的转动方向问题，注意单位换算问题（线速度<->角速度 弧度<->角度）

<br><br>
### 2、双环pid（pid.c/h）
双环pid主要是位置环（外环）和速度环（内环）两个。
- 位置环输入的是目标位置，输出的是到达目标位置所需要的速度值（全局坐标系），反馈使用的是运动正解算出的位姿及传感器位姿。 
- 速度环输入的是电机的目标速度（小车坐标系），输出的是电机的电流值，反馈使用的是电机反馈回来的报文。 
如果觉得描述的不清晰，可以看下面这个流程图：  
<img width="1429" height="300" alt="image" src="https://github.com/user-attachments/assets/ae1c15e9-4ee9-4a8a-b1ff-1a10c8dbef79" />  
由于使用的是freertos，所以大多使用任务描述，裸机也可以实现，合理使用定时器按照流程时序来写就好了，注意外环频率要比内环低。图中的运动正解任务中包含了坐标轴转换，但未标出。

<br><br>
### 3、任务框架（freertos.c）/流程控制 （Process_Control.c）
由于双环pid作为任务在后台一直运行，故仅需改变位置环pid的目标值即可完成整车的运动控制，机械臂同理。在流程控制任务中会读取任务流程（一个结构体数组），根据每一个流程的要求更改各个pid的目标值完成运动控制，同时根据设定延时运行下一个流程。
执行流程及任务框图如下所示：  
<img width="249" height="904" alt="image" src="https://github.com/user-attachments/assets/3b27fab4-5df8-4ec8-a41b-47296b35f6c0" />
<img width="1716" height="810" alt="image" src="https://github.com/user-attachments/assets/aa09ab6b-d2cd-4978-9435-e212a586a84a" />
<br><br>
## 六、部分补充内容
1. 为了加速pid计算，f4开启了fpu，使用了dsp库，使用了O3优化，具体方法可以看我的博客：[https://blog.csdn.net/Horizon_bendo/article/details/148482310]
2. 引脚定义参见项目根目录ioc文件
3. 项目中残存部分弃用驱动，已验证但仅供参考



