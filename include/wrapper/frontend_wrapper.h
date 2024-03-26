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
		AVIA = 0,
		VELO = 1
	};
	class FrontEndWrapper
	{
	public:
		 FrontEndWrapper(ros::NodeHandle &nh);
		~ FrontEndWrapper();

	private:
		Slam::FrontEnd::Ptr front_end_ptr;
		ros::Subscriber cloud_subscriber;
		ros::Subscriber imu_subscriber;
		ros::Publisher curr_cloud_pub;
		ros::Publisher path_pub;
		ros::Publisher local_map_pub;
		std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;

		//�ص�����
		void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr& msg);
		void imuMsgCallBack(const sensor_msgs::ImuPtr& msg);
		void run();
		void publishMsg();
	};
}