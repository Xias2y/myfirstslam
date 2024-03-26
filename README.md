# ieskf
##工程框架
slam
 ├── CMakeLists.txt
 ├── package.xml
 ├── launch
 ├── include  头文件
 │   ├── slam  算法部分
 │   │    ├── math  计算
 │   │    ├── modules  模块
 │   │    │    ├──frontbackPropagate  向前/向后传播
 │   │    │    ├──frontend  前端
 │   │    │    ├──ieskf  滤波
 │   │    │    └──map  地图管理
 │   │    └── type
 │   └── wrapper  ros部分
 │       └── lidar_process  雷达适配
 └── src  实现
     ├── app
     │   └── ros_humble
     ├── ieskf
     │    ├──frontbackPropagate
     │    ├──frontend
     │    ├──ieskf
     │    └──map
     └── wrapper
