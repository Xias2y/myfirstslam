#pragma once
#include "slam/type/imu.h"
#include "slam/type/pointcloud.h"
#include <deque>

namespace Slam
{
	struct MeasureGroup
	{
		double lidar_begin_time;
		double lidar_end_time;
		std::deque<IMU> imus;
		PointCloud cloud;
	};
}