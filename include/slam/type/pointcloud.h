#pragma once
#include "slam/type/point.h"
#include "slam/type/timestamp.h"
namespace Slam
{
	using PCLPointCloud = pcl::PointCloud<Point>;//定义类型别名
	using PCLPointCloudPtr = PCLPointCloud::Ptr;//智能指针
	using PCLPointCloudConstPtr = PCLPointCloud::ConstPtr;//外部对局部地图只读
	struct PointCloud
	{
		using Ptr = std::shared_ptr<PointCloud>;//指向此结构体的共享指针
		TimeStamp time_stamp;
		PCLPointCloudPtr cloud_ptr;
		PointCloud() { //构造函数
			cloud_ptr = pcl::make_shared<PCLPointCloud>();
		}
	};
}