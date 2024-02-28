//自定义PCL点云类型参考：https://zhuanlan.zhihu.com/p/458373010

#pragma once
#ifndef PCL_NO_PRECOMPILE  //是否被宏定义过
#define PCL_NO_PRECOMPILE //PCL1.7开始，需要在包含任何PCL头文件之前定义此项，来包含模板化算法
#endif

#include <pcl/impl/pcl_base.hpp>
#include <pcl/pcl_base.h>
#include "pcl/point_types.h"  //非标准头文件
#include <pcl/point_cloud.h>

//SSE支持128bit的多指令运行，处理对象必须在内存地址以16byte整数倍的地方开始
namespace Slam {
	struct EIGEN_ALIGN16 Point {  //强制SSE（向量化运算）填充以对齐内存
		PCL_ADD_POINT4D;  //添加xyz和填充类型的首选方式
		float intensity;
		std::uint32_t offset_time;  //时间戳
		std::uint32_t ring;  //点云所属线束
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW //重载new函数，对齐地址
	};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(Slam::Point,  //点云类型注册
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, intensity, intensity)
	(std::uint32_t, offset_time, offset_time)
	(std::uint32_t, ring, ring)
)