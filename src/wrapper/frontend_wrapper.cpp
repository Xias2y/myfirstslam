#include "wrapper/frontend_wrapper.h"
#include "slam/globaldeine.h"
#include "pcl/common/transforms.h"

namespace ROSNoetic
{
	FrontEndWrapper::FrontEndWrapper(ros::NodeHandle& nh)
	{
		std::string config_file_name, lidar_topic, imu_topic;
		nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
		nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
		nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
		std::cout << lidar_topic << std::endl;
		std::cout << imu_topic << std::endl;
		//使用CONFIG_DIR定义的路径作为配置文件的目录
		//CONFIG_DIR在之前的globaldefine里，即/config/
		//用这个路径创建了slam：：frontend对象
		front_end_ptr = std::make_shared<Slam::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");


		//发布者和订阅者
		cloud_subscriber = nh.subscriber(lidar_topic, 100, &FrontEndWrapper::lidarCloudMsgCallBack, this);
		
	}
}