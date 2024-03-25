#include "wrapper/frontend_wrapper.h"
#include "slam/globaldeine.h"
#include "pcl/common/transforms.h"

namespace ROSNoetic
{
	FrontEndWrapper::FrontEndWrapper(ros::NodeHandle& nh)
	{
		std::string config_file_name, lidar_topic, imu_topic;
		//通过ros参数服务器读入配置文件
		nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
		nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
		nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
		//使用CONFIG_DIR定义的路径作为配置文件的目录
		//CONFIG_DIR在之前的globaldefine里，即/config/
		//创建了一个指向 Slam::FrontEnd 类型对象的共享指针，并将其赋值给了 front_end_ptr
		front_end_ptr = std::make_shared<Slam::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");

		//发布者和订阅者（100表示消息队列的大小）
		cloud_subscriber = nh.subscriber(lidar_topic, 100, &FrontEndWrapper::lidarCloudMsgCallBack, this);
		imu_subscriber = nh.subscribe(imu_topic, 100, &FrontEndWrapper::imuMsgCallBack, this);

		//读取雷达类型
		int lidar_type = 0;
		nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);//参数名称，存储变量，默认值
		if (lidar_type == AVIA) {
			lidar_process_ptr = std::make_shared<AVIAProcess>();
		}
		else if (lidar_type == VELO) {
			lidar_process_ptr = std::make_shared<VelodyneProcess>();
		}
		else {
			std::cout << "unsupport lidar type" << std::endl;
			exit(100);
		}
		//ros发布器，发布话题为curr_cloud的话题
		curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);
		path_pub = nh.advertise<nav_msgs::Path>("path", 100);
		local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_map", 100);
		run();
	}
	FrontEndWrapper::~FFrontEndWrapper()
	{
	}

	void FrontEndWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr& msg) {
		Slam::PointCloud cloud;
		lidar_process_ptr->process(*msg, cloud);//在avia中，处理点云数据并转到cloud
		front_end_ptr->addPointCloud(cloud);
	}

	void FrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr& msg) {
		Slam::IMU imu;
		//秒化为纳秒
		imu.time_stamp.fromNsec(msg->header.stamp.toNSec());

		//加速度赋值
		imu.acceleration = { msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z };
		//角速度赋值
		imu.gyroscope = { msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z };
		front_end_ptr->addImu(imu);
	}

	void FrontEndWrapper::run() {
		ros::Rate rate(500);//500hz的ros频率对象rate
		while (ros::ok()) {
			rate.sleep();//等待以达到频率
			ros::spinOnce();//处理所有的回调函数
			if (front_end_ptr->track()) {//如果时间戳统一则发布
				publishMsg();
			}
		}
	}

	void FrontEndWrapper::publishMsg() {
		static nav_msgs::Path path;
		auto X = front_end_ptr->readState();//转换为世界坐标系的点云
		path.header.frame_id = "map";
		geometry_msgs::PoseStamped psd;
		psd.pose.position.x = X.position.x();
		psd.pose.position.y = X.position.y();
		psd.pose.position.z = X.position.z();
		path.poses.push_back(psd);
		path_pub.publish(path);
		IESKFSlam::PCLPointCloud cloud = front_end_ptr->readCurrentPointCloud();
		pcl::transformPointCloud(
			cloud, cloud, IESKFSlam::compositeTransform(X.rotation, X.position).cast<float>());
		// auto cloud =front_end_ptr->readCurrentPointCloud();
		sensor_msgs::PointCloud2 msg;
		pcl::toROSMsg(cloud, msg);
		msg.header.frame_id = "map";
		curr_cloud_pub.publish(msg);

		cloud = front_end_ptr->readCurrentLocalMap();
		pcl::toROSMsg(cloud, msg);
		msg.header.frame_id = "map";
		local_map_pub.publish(msg);
	}
}
