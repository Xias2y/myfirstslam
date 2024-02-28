#pragma once
#include "slam/modules/module_base.h"
#include "slam/type/imu.h"
#include "slam/type/base_type.h"
#include "slam/type/pose.h"

namespace Slam
{
	class FrontEnd: private ModuleBase
	{
	public:
		using Ptr = std::shared_ptr<FrontEnd>;//shared_ptr：智能指针
	private:
		std::deque<IMU> imu_deque;//deque：STL容器，双向开口的连续线性空间（vector为单向开口的内存空间）
		std::deque<PointCloud> pointcloud_deque;
		std::deque<Pose> pose_deque;
		PCLPointCloud current_pointcloud;//世界坐标系下的点云
	public:
		FrontEnd(const std::string & config_file_path, const std::string & prefix);
		~FrontEnd();
		//添加数据
		void addImu(const IMU& imu);
		void addPointCloud(const PointCloud& pointcloud);
		void addPose(const Pose& pose);
		//追踪
		bool track();
		//w坐标系点云读取
		const PCLPointCloud& readCurrentPointCloud();
	};
}