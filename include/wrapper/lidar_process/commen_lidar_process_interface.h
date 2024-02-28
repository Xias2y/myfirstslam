#pragma once
#include "slam/type/base_type.h"
#include <sensor_msgs/PointCloud2.h>//ros�е���Ϣ����
#include "pcl_conversions/pcl_conversions.h"//ros��pcl��������ת��

namespace Slam
{
	class CommonLidarProcessInterface
	{
	public:
		virtual bool process(const sensor_msgs::PointCloud2& msg, Slam::PointCloud& cloud) = 0;
	};
}