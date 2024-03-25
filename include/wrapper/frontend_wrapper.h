#pragma once
#include "slam/globaldefine.h"
#include "slam/modules/frontend/frontend.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "wrapper/frontend_wrapper.h"
#include "wrapper/lidar_process/avia_process.h"
#include "wrapper/lidar_process/velodyne_process.h"

namespace ROSNoetic
{
	enum LIDAR_TYPE {
		AVIA = 0;
		VELO = 1;
	};
	class FrontEndWrapper
	{
	public:
		 FrontEndWrapper(ros::NodeHandle &nh);
		~ FrontEndWrapper();

	private:
		Slam::FrontEnd::Ptr front_end_ptr;
		//订阅点云、imu、里程计，发布当前点云
		ros::Subscriber cloud_subscriber;
		ros::Subscriber imu_subscriber;
		ros::Subscriber odometry_subscriber;
		ros::Publisher curr_cloud_pub;

		std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;
		
		Slam::PCLPointCloud curr_cloud;
		Eigen::Quaterniond curr_q;
		Eigen::Vector3d curr_t;
		//回调函数
		void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr& msg);
		void imuMsgCallBack(const sensor_msgs::ImuPtr& msg);
		void odometryMsgCallBack(const nav_msgs::OdometryPtr& msg);
		void run();
		void publishMsg();
	};
}