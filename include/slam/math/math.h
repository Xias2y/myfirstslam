#pragma once
#include <Eigen/Dense>

namespace Slam
{
	//���ϱ任������Ԫ������ά������
	static Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
		Eigen::Matrix4d ans;
		ans.setIdentity();
		ans.block<3, 3>(0, 0) = q.toRotationMatrix();//����Ԫ��qת��Ϊ��ת���󣬰���ת���ָ�ֵ
		ans.block<3, 1>(0, 3) = t;//��ƽ�Ʋ��ָ�ֵ
		return ans;
	}
}