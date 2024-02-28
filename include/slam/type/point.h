//�Զ���PCL�������Ͳο���https://zhuanlan.zhihu.com/p/458373010

#pragma once
#ifndef PCL_NO_PRECOMPILE  //�Ƿ񱻺궨���
#define PCL_NO_PRECOMPILE //PCL1.7��ʼ����Ҫ�ڰ����κ�PCLͷ�ļ�֮ǰ������������ģ�廯�㷨
#endif

#include <pcl/impl/pcl_base.hpp>
#include <pcl/pcl_base.h>
#include "pcl/point_types.h"  //�Ǳ�׼ͷ�ļ�
#include <pcl/point_cloud.h>

//SSE֧��128bit�Ķ�ָ�����У��������������ڴ��ַ��16byte�������ĵط���ʼ
namespace Slam {
	struct EIGEN_ALIGN16 Point {  //ǿ��SSE�����������㣩����Զ����ڴ�
		PCL_ADD_POINT4D;  //���xyz��������͵���ѡ��ʽ
		float intensity;
		std::uint32_t offset_time;  //ʱ���
		std::uint32_t ring;  //������������
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW //����new�����������ַ
	};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(Slam::Point,  //��������ע��
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, intensity, intensity)
	(std::uint32_t, offset_time, offset_time)
	(std::uint32_t, ring, ring)
)