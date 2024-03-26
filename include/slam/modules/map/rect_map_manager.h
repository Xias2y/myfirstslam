#pragma once
#include "slam/type/pointcloud.h"
#include "slam/modules/module_base.h"
#include "pcl/common/transforms.h"
#include "slam/math/math.h"
#include "slam/type/base_type.h"

namespace Slam
{
	class RectMapManager:private ModuleBase
	{
	public:
		RectMapManager(const std::string& config_file_path, const std::string& prefix);
		~RectMapManager();
		void reset();
		void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond& att_q, const Eigen::Vector3d& pos_t);
		PCLPointCloudConstPtr getLocalMap();//读取局部地图
		KDTreeConstPtr readKDtree();

	private:
		PCLPointCloudPtr local_map_ptr;
		KDTreePtr kdtree_ptr;
		float map_side_length_2;//地图边长的一半
		float map_resolution;
	};

}