# 2023哨兵

## 修改内容及问题
1.camera包内增加了小相机的读取和话题发送

2.rm_interfaces包内增加了两个消息，一种发布原始的识别后的数据，第二种是第一种的数组形式，考虑到会有一个相机识别到多辆车

3.omni中python文件负责接受img和做识别，c++中主要是整合，目前只能有大概的方向，解算不精确，还没写发布的部分，未确定发布消息的格式

4.小相机暂时修改不了参数


### 编译
```shell
#先编译依赖EigenRand
cd src/EigenRand
mkdir build
cd build/
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j16
#再编译ros软件包
cd aqs_dwr2023-sentry
catkin_make -DCATKIN_WHITELIST_PACKAGES="rm_interfaces;cv_bridge_new" # 先单独编译这两个包
catkin_make -DCATKIN_WHITELIST_PACKAGES="" # 再编译剩下的所有包
```

### 运行
```shell
source ./devel/setup.bash
roslaunch rm_bringup bringup.launch
```

### 可视化界面
```shell
source ./devel/setup.bash
roslaunch rm_bringup rviz.launch
```


# FYT2023-哨兵视觉项目介绍

## 项目框架
  项目整体跟从FYT2023自瞄框架，但做了部分改动,目前只实现哨兵自瞄功能

1.解耦 

哨兵项目去掉了自瞄程序中的config节点，目的是想真正实现各个功能能独立运行，更方便扩展和修改

>所有参数，除了config/general_param.yaml中定义的几个，其他全部放在对应软件包下的config文件夹内。所有参数从ros的param_server中读取

2.对部分软件包做了修改

>fyt_msgs改为rm_interfaces，img_catcher改为camera，solve_armor改为post_processor

重写了serial节点（基于开源项目rmoss中提供的rm_base组件），修改了solve_armor节点，不再订阅图像话题

3.修改了部分msg

4.重新定义了串口协议

下位机发送敌人颜色，射速（float32)，俯仰角(float32)，偏航角(float32)

上位机发送目标俯仰角(float32)，目标偏航角(float32)，距离(flaot32)

5.自瞄可视化

使用rviz可视化，后面会在自瞄项目中也添加

## 更新日志

```
12.18 更新四点模型推理,C++的onnx-runtime和Python的Tensorrt，在我的电脑上测试，onnx-runtime约100fps，tensorrt约140fps
因为大部分设备还没配置onnx-runtime和tensorrt的环境，默认还是用Darknet推理，Darknet不能归回四点，所以要传统识别四点
```

## 近期目标 （寒假前后）
1.大致实现全向识别

2.新的反小陀螺算法(已完成)

>首先假设我们正在跟踪模式，如果我们发现，除了我们正在跟踪的装甲板外还有一块相同id的装甲板，并且我们现在跟踪的这块装甲板的角度已经超过30度了（快要看不见另一个灯条了），
>认为敌人处于小陀螺模式，此时切换跟踪目标为视野里的另一块装甲板


3.扩展卡尔曼滤波 基于CTRV运动模型（已完成）