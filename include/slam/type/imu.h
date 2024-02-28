#pragma once
#include <Eigen/Dense>
#include "slam/type/timestamp.h"
namespace Slam
{
	class IMU
	{
	public:
		//eigen���еľ�̬��Ա����������һ����ά����������Ϊ0
		Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();//���ٶ�
		Eigen::Vector3d gyroscope = Eigen::Vector3d::Zero();//������
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
		//��Ԫ�����������������ķǳ�Ա������������������˽�г�Ա�ͱ�����Ա
		//��ʽ���
		friend std::ostream& operator<<(std::ostream& ostream, const IMU& imu) {
			ostream << "imu_time:" << imu.time_stamp.sec() << "s | imu_acc" << imu.acceleration.transpose() << " | imu_gro:" << imu.gyroscope.transpose();
			return ostream;
		}
	};
}