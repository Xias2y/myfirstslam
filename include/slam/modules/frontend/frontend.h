#pragma once
#include "slam/globaldefine.h"
#include "slam/type/imu.h"
#include "slam/type/base_type.h"
#include "slam/type/pose.h"
#include "slam/type/measure_group.h"
#include "slam/modules/module_base.h"
#include "slam/modules/ieskf/ieskf.h"
#include "slam/modules/map/rect_map_manager.h"
#include "slam/modules/frontbackPropagate/frontback_propagate.h"
#include "ieskf_slam/modules/frontend/lio_zh_model.h"

namespace Slam
{
	class FrontEnd: private ModuleBase
	{
	public:
		using Ptr = std::shared_ptr<FrontEnd>;//shared_ptr：智能指针
	private:
		std::deque<IMU> imu_deque;//deque：STL容器，双向开口的连续线性空间（vector为单向开口的内存空间）
		std::deque<PointCloud> pointcloud_deque;
		std::shared_ptr<IESKF> ieskf_ptr;
		std::shared_ptr<RectMapManager> map_ptr;
		std::shared_ptr<FrontbackPropagate> fbpropagate_ptr;
		VoxelFilter voxel_filter;
		LIOZHModel::Ptr lio_zh_model_ptr;
		PCLPointCloudPtr filter_point_cloud_ptr;;//世界坐标系下的点云
		bool imu_inited = false;
		double imu_scale = 1;//缩放系数
		Eigen::Quaterniond extrin_r;
		Eigen::Vector3d extrin_t;

		bool enable_record = false;
		std::string record_file_name;
		std::fstream record_file;

	public:
		FrontEnd(const std::string & config_file_path, const std::string & prefix);
		~FrontEnd();
		//添加数据
		void addImu(const IMU& imu);
		void addPointCloud(const PointCloud& pointcloud);

		//追踪
		bool track();
		//w坐标系点云读取
		const PCLPointCloud& readCurrentPointCloud();
		//获得一帧点云和点云中包含的imu
		const PCLPointCloud& readCurrentLocalMap();
		bool syncMeasureGroup(MeasureGroup& mg);
		void initState(MeasureGroup& mg);

		IESKF::State18 readState();
	};
}