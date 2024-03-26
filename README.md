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
 
 	│        ├── slam  算法部分
 
 	│        │         ├── math  计算
 
 	│        │         ├── modules  模块
 
 	│        │         │         ├──frontbackPropagate  向前/向后传播
 
 	│        │         │         ├──frontend  前端
 
 	│        │         │         ├──ieskf  滤波

 	│        │         │         └──map  地图管理
 
 	│        │         └── type
 
 	│        └── wrapper  ros部分
 
 	│                 └── lidar_process  雷达适配
 
 	└── src  实现
 
               ├── app
     
               │        └── ros_humble
     
               ├── ieskf
     
               │         ├──frontbackPropagate
               
               │         ├──frontend
          
               │         ├──ieskf
     
               │         └──map
     
               └── wrapper
	       
# 使用说明
	如果你的配置满足fast-lio系列算法，可以成功编译运行，则不需要额外进行配置

## Ubuntu和ros
	Ubuntu >= 16.04
 
 	PCL >= 1.8

  	Eigen >= 3.3.4
	
