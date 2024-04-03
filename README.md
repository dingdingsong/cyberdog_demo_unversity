# 铁蛋ROS2开发示例


## 1、简述
**本项目主要示例了如何开发一个调用铁蛋语音、运动等能力的ROS2功能包。**

铁蛋项目基于ROS2的galactic版本进行开发。项目已开源至：https://github.com/MiRoboticsLab/cyberdog_ws。 进行项目开发前可以详细浏览项目博客：https://miroboticslab.github.io/blogs/#/。 进行铁蛋项目二次开发需要具备一定的代码开发能力和ROS2的基础知识。

阅读铁蛋项目博客和项目源码时，可优先关注铁蛋项目的**主要功能和ROS接口**以便于快速上手开发调试。关键接口： https://miroboticslab.github.io/blogs/#/cn/developer_guide

## 2、开发
**主要流程为：**     
    1）创建ROS2功能包；     
    2）源码开发、CMakeLists.txt、package.xml文件编写；   
    3）源码编译（本机编译或docker镜像中编译）；   
    4）将编译产物放到铁蛋上运行查看效果；

**关于源码开发的一些说明**  
1）头文件，主要包括：ROS相关的头文件、ROS接口相关头文件、C++及其他第三方库头文件等。**如果要使用ROS的相关话题、服务等，必须导入ROS自带的头文件或铁蛋项目的自定义头文件**；  
2）ROS节点开发可参考：https://github.com/ros2/demos/tree/galactic/demo_nodes_cpp或已开源的铁蛋项目中的ROS功能包；  
3）CMakeLists编写可参考：https://docs.ros.org/en/galactic/How-To-Guides/Ament-CMake-Documentation.html 或 https://docs.ros.org/en/galactic/How-To-Guides/Ament-CMake-Python-Documentation.html 。基于C++开发的ROS功能包参考第一条链接，基于python开发的ROS功能包参考第二条链接。  
4）**铁蛋的AI相机、realease相机、鱼眼相机等默认情况下不会产生数据**，需要通过服务请求或lifecycle的状态设置来打开，打开相机后才会有图像数据发布出来。  

## 3、编译与运行
编译运行指令如下：

```
// 1.编译
colcon build --merge-install --packages-up-to cyberdog_example

// 2.编译完成，source环境变量
source install/setup.bash

// 3.运行节点
ros2 run cyberdog_example cyberdog_example -ros-args -r __ns:=/`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`
```

说明：-ros-args -r __ns:=/`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"` 的作用是添加命名空间，以保证cyberdog_example节点能与NX板上已运行起来的ROS节点进行数据交互


其他一些指令说明:
```
// 查看ros接口
ros2 interface show **

// 例如查看Image接口
ros2 interface show sensor_msgs/msg/Image

// 查看NX上某个节点的日志信息
journal -u cyberdog_bringup.service -fn 100000 |grep  <node_name>
journal -u cyberdog_sudo.service -fn 100000 |grep <node_name>
```
