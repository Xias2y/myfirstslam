#pragma once
#include <Eigen/Dense>

namespace Slam
{
	//复合变换矩阵（四元数，三维向量）
	static Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
		Eigen::Matrix4d ans;
		ans.setIdentity();
		ans.block<3, 3>(0, 0) = q.toRotationMatrix();//将四元数q转换为旋转矩阵，把旋转部分赋值
		ans.block<3, 1>(0, 3) = t;//把平移部分赋值
		return ans;
	}
}