# Webots_ROS_Darwin_op3
Using ROS to control webots Darwin_op3 in Webots

## 仿真环境配置与插件安装
### 1. ROS(melodic)安装

参见http://wiki.ros.org/melodic/Installation/Ubuntu

### 2. Webots(2023b)安装与测试


``` Bash
sudo apt install ./webots_2023b_amd64.deb

webots

echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc

source ~/.bashrc
```
### 3. webots_ros安装

``` Bash
sudo apt install ros-noetic-webots-ros
```

``` Bash
roslaunch webots_ros e_puck_line.launch
```
### 4. multiplot安装

``` Bash
sudo apt install ros-noetic-rqt-multiplot
```
### 5. QtCreator(待完善)
请参考： 
https://blog.csdn.net/YMGogre/article/details/130588657

## ROS工程编译与依赖安装


### 1.单独编译消息包
``` Bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="robotis_controller_msgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES="op3_walking_module_msgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```


## 运行

### 1. 拉起Webots和相应的控制器
```bash
roslaunch op3_webots_controller op3_webots_motion.launch 
```


### 2. 拉起控制页面


```bash
rosrun op3_webots_gui op3_webots_gui
```