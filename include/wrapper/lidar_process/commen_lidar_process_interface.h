#pragma once
#include "slam/type/base_type.h"
#include <sensor_msgs/PointCloud2.h>//ros中的消息类型
#include "pcl_conversions/pcl_conversions.h"//ros和pcl数据类型转换

namespace Slam
{
	class CommonLidarProcessInterface
	{
	public:
		virtual bool process(const sensor_msgs::PointCloud2& msg, Slam::PointCloud& cloud) = 0;
	};
}