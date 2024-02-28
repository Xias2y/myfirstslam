#pragma once
#include <Eigen/Dense>
#include "timestamp.h"
namespace Slam
{
	struct Pose
	{
		TimeStamp time_stamp;
		Eigen::Quaterniond rotation;//��Ԫ����ʾ��ת
		Eigen::Vector3d position;//λ��
	};
}