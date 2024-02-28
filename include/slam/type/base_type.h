#pragma once
#include "slam/type/pointcloud.h"
#include <pcl/filters/voxel_grid.h>//�����˲���ͷ�ļ�
#include <pcl/kdtree/kdtree_flann.h>
namespace Slam
{
	using VoxelFilter = pcl::VoxelGrid<Point>;//�����˲�
	using KDTree = pcl::KDTreeFLANN<Point>;//KDTree
	using KDTreePtr = KDTree::Ptr;
	const double GRAVITY = 9.81;//��������
}