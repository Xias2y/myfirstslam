# 项目介绍
本项目是基于fast-lio论文进行解耦合式复写的里程计框架，使用并行kdtree，取消了特征提取步骤，使用全部原始点云进行匹配
 
 目前适配avia和velodyne雷达
 
 最终点云建图的显示效果接近于fast-lio
  
# 工程框架
 slam

 ├── CMakeLists.txt
 
 ├── package.xml
 
 ├── launch
 
 ├── include  头文件
 
 │      &emsp;&emsp;  ├── slam  算法部分
 
 │     &emsp;&emsp;   │    &emsp;&emsp;     ├── math  计算
 
 │     &emsp;&emsp;   │      &emsp;&emsp;   ├── modules  模块
 
 │      &emsp;&emsp;  │      &emsp;&emsp;   │     &emsp;&emsp;    ├──frontbackPropagate  向前/向后传播
 
 │     &emsp;&emsp;   │     &emsp;&emsp;    │     &emsp;&emsp;    ├──frontend  前端
 
 │      &emsp;&emsp;  │      &emsp;&emsp;   │      &emsp;&emsp;   ├──ieskf  滤波

 │      &emsp;&emsp;  │       &emsp;&emsp;  │     &emsp;&emsp;    └──map  地图管理

 │      &emsp;&emsp;  │      &emsp;&emsp;   └── type
 
 │      &emsp;&emsp;  └── wrapper  ros部分
 
 │               &emsp;&emsp;  └── lidar_process  雷达适配
 
 └── src  实现
 
 │    &emsp;&emsp;   ├── app
     
 │     &emsp;&emsp;  │     &emsp;&emsp;   └── ros_humble
 
 │    &emsp;&emsp;   ├── ieskf
     
 │     &emsp;&emsp;  │      &emsp;&emsp;   ├──frontbackPropagate
               
 │    &emsp;&emsp;   │      &emsp;&emsp;  ├──frontend
          
 │     &emsp;&emsp;  │     &emsp;&emsp;    ├──ieskf
     
 │     &emsp;&emsp;  │      &emsp;&emsp;   └──map
     
 │    &emsp;&emsp;   └── wrapper
	       
# 使用说明
如果你的配置满足fast-lio系列算法，可以成功编译运行，则不需要额外进行配置

## Ubuntu和ros
Ubuntu >= 16.04
  
PCL >= 1.8

Eigen >= 3.3.4

livox_ros_driver支持
## 工程构建
   	cd ~/$A_ROS_DIR$/src
  	git clone https://github.com/Xias2y/myfirstslam.git
 	cd myfirstslam
 	git submodule update --init
  	cd ../..
  	source devel/setup.bash
  	catkin_make
   	 
## 运行
	roslaunch slam avia.launch
	rosbag play YOUR_DOWNLOADED.bag


