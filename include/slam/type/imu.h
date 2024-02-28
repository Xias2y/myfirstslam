#pragma once
#include <Eigen/Dense>
#include "slam/type/timestamp.h"
namespace Slam
{
	class IMU
	{
	public:
		//eigen库中的静态成员函数，创建一个三维向量，分量为0
		Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();//加速度
		Eigen::Vector3d gyroscope = Eigen::Vector3d::Zero();//陀螺仪
		TimeStamp time_stamp;
		void clear() {
			acceleration = Eigen::Vector3d::Zero();
			gyroscope = Eigen::Vector3d::Zero();
			time_stamp = 0;
		}
		IMU operator + (const IMU& imu) {
			IMU res;
			res.acceleration = this->acceleration + imu.acceleration;
			res.gyroscope = this->gyroscope + imu.gyroscope;
			return res;
		}
		IMU operator *(double k) {
			IMU res;
			res.acceleration = this->acceleration *k;
			res.gyroscope = this->gyroscope *k;
			return res;
		}
		IMU operator /(double k) {
			IMU res;
			res.acceleration = this->acceleration /k;
			res.gyroscope = this->gyroscope /k;
			return res;
		}
		//友元函数：在类中声明的非成员函数，被允许访问类的私有成员和保护成员
		//流式输出
		friend std::ostream& operator<<(std::ostream& ostream, const IMU& imu) {
			ostream << "imu_time:" << imu.time_stamp.sec() << "s | imu_acc" << imu.acceleration.transpose() << " | imu_gro:" << imu.gyroscope.transpose();
			return ostream;
		}
	};
}