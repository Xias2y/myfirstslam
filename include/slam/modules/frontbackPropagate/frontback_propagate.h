#pragma once
#include "slam/modules/ieskf/ieskf.h"
#include "slam/type/measure_group.h"

namespace Slam
{
	class FrontbackPropagate
	{
	public:
		double last_lidar_end_time_;
		IMU last_imu_;
		double imu_scale;
		IMU last_imu;
		FrontbackPropagate(/*args*/);
		~FrontbackPropagate();
		void propagate(MeasureGroup& mg, IESKF::Ptr ieskf_ptr);

	private:
		struct IMUPose6d
		{
			double time;
			Eigen::Vector3d acc; // 加速度
			Eigen::Vector3d angvel; // 角速度
			Eigen::Vector3d vel; // 速度
			Eigen::Vector3d pos; // 位置
			Eigen::Quaterniond rot; // 旋转
			IMUPose6d(double time_ = 0, Eigen::Vector3d a_ = Eigen::Vector3d::Zero(),
				Eigen::Vector3d av_ = Eigen::Vector3d::Zero(),
				Eigen::Vector3d v_ = Eigen::Vector3d::Zero(),
				Eigen::Vector3d p_ = Eigen::Vector3d::Zero(), 
				Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity())
			{
				time = time_;
				acc = a_;
				angvel = av_;
				vel = v_;
				pos = p_;
				rot = q_;
			}
		};
		Eigen::Vector3d acc_s_last;
		Eigen::Vector3d angvel_last;
	};
}