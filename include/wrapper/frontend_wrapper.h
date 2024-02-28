#pragma once
#include "slam/modules/frondend/frondend.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "wrapper/lidar_process/avia_process.h"

namespace ROSNoetic
{
	enum LIDAR_TYPE {
		AVIA = 0;//���岻ͬ�״�����
	};
	class FrontEndWrapper
	{
	public:
		 FrontEndWrapper(ros::NodeHandle &nh);
		~ FrontEndWrapper();

	private:
		Slam::FrontEnd::Ptr front_end_ptr;
		//���ĵ��ơ�imu����̼ƣ�������ǰ����
		ros::Subscriber cloud_subscriber;
		ros::Subscriber imu_subscriber;
		ros::Subscriber odometry_subscriber;
		ros::Publisher curr_cloud_pub;

		std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;
		
		Slam::PCLPointCloud curr_cloud;
		Eigen::Quaterniond curr_q;
		Eigen::Vector3d curr_t;
		//�ص�����
		void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr& msg);
		void imuMsgCallBack(const sensor_msgs::ImuPtr& msg);
		void odometryMsgCallBack(const nav_msgs::OdometryPtr& msg);
		void run();
		void publishMsg();
	};
}