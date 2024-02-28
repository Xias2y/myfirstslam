#pragma once
#include <Eigen/Dense>
#include "timestamp.h"
namespace Slam
{
	struct Pose
	{
		TimeStamp time_stamp;
		Eigen::Quaterniond rotation;//四元数表示旋转
		Eigen::Vector3d position;//位置
	};
}