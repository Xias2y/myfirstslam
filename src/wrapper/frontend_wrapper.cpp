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
		//ʹ��CONFIG_DIR�����·����Ϊ�����ļ���Ŀ¼
		//CONFIG_DIR��֮ǰ��globaldefine���/config/
		//�����·��������slam����frontend����
		front_end_ptr = std::make_shared<Slam::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");


		//�����ߺͶ�����
		cloud_subscriber = nh.subscriber(lidar_topic, 100, &FrontEndWrapper::lidarCloudMsgCallBack, this);
		
	}
}