#pragma once
#include "slam/type/point.h"
#include "slam/type/timestamp.h"
namespace Slam
{
	using PCLPointCloud = pcl::PointCloud<Point>;//�������ͱ���
	using PCLPointCloudPtr = PCLPointCloud::Ptr;//����ָ��
	using PCLPointCloudConstPtr = PCLPointCloud::ConstPtr;//�ⲿ�Ծֲ���ͼֻ��
	struct PointCloud
	{
		using Ptr = std::shared_ptr<PointCloud>;//ָ��˽ṹ��Ĺ���ָ��
		TimeStamp time_stamp;
		PCLPointCloudPtr cloud_ptr;
		PointCloud() { //���캯��
			cloud_ptr = pcl::make_shared<PCLPointCloud>();
		}
	};
}