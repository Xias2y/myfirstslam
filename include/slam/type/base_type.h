#pragma once
#include "slam/type/pointcloud.h"
#include <pcl/filters/voxel_grid.h>//�����˲���ͷ�ļ�
#include <pcl/kdtree/kdtree_flann.h>

namespace Slam
{
	using VoxelFilter = pcl::VoxelGrid<Point>;//�����˲�
	using KDTree = pcl::KDTreeFLANN<Point>;//KDTree
	using KDTreePtr = KDTree::Ptr;
	using KDTreeConstPtr = KDTree::ConstPtr;

	const double GRAVITY = 9.81;//��������

	template<typename _first, typename _second, typename _thrid>
	struct triple {
		_first first;
		_second second;
		_thrid thrid;
	};
}