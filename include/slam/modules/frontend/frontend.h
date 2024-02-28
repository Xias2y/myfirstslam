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
		using Ptr = std::shared_ptr<FrontEnd>;//shared_ptr������ָ��
	private:
		std::deque<IMU> imu_deque;//deque��STL������˫�򿪿ڵ��������Կռ䣨vectorΪ���򿪿ڵ��ڴ�ռ䣩
		std::deque<PointCloud> pointcloud_deque;
		std::deque<Pose> pose_deque;
		PCLPointCloud current_pointcloud;//��������ϵ�µĵ���
	public:
		FrontEnd(const std::string & config_file_path, const std::string & prefix);
		~FrontEnd();
		//�������
		void addImu(const IMU& imu);
		void addPointCloud(const PointCloud& pointcloud);
		void addPose(const Pose& pose);
		//׷��
		bool track();
		//w����ϵ���ƶ�ȡ
		const PCLPointCloud& readCurrentPointCloud();
	};
}