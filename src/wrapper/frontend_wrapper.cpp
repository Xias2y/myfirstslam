#include "wrapper/frontend_wrapper.h"
#include "slam/globaldeine.h"
#include "pcl/common/transforms.h"

namespace ROSNoetic
{
	FrontEndWrapper::FrontEndWrapper(ros::NodeHandle& nh)
	{
		std::string config_file_name, lidar_topic, imu_topic;
		//ͨ��ros�������������������ļ�
		nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
		nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
		nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
		//ʹ��CONFIG_DIR�����·����Ϊ�����ļ���Ŀ¼
		//CONFIG_DIR��֮ǰ��globaldefine���/config/
		//������һ��ָ�� Slam::FrontEnd ���Ͷ���Ĺ���ָ�룬�����丳ֵ���� front_end_ptr
		front_end_ptr = std::make_shared<Slam::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");

		//�����ߺͶ����ߣ�100��ʾ��Ϣ���еĴ�С��
		cloud_subscriber = nh.subscriber(lidar_topic, 100, &FrontEndWrapper::lidarCloudMsgCallBack, this);
		imu_subscriber = nh.subscribe(imu_topic, 100, &FrontEndWrapper::imuMsgCallBack, this);

		//��ȡ�״�����
		int lidar_type = 0;
		nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);//�������ƣ��洢������Ĭ��ֵ
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
		//ros����������������Ϊcurr_cloud�Ļ���
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
		lidar_process_ptr->process(*msg, cloud);//��avia�У�����������ݲ�ת��cloud
		front_end_ptr->addPointCloud(cloud);
	}

	void FrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr& msg) {
		Slam::IMU imu;
		//�뻯Ϊ����
		imu.time_stamp.fromNsec(msg->header.stamp.toNSec());

		//���ٶȸ�ֵ
		imu.acceleration = { msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z };
		//���ٶȸ�ֵ
		imu.gyroscope = { msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z };
		front_end_ptr->addImu(imu);
	}

	void FrontEndWrapper::run() {
		ros::Rate rate(500);//500hz��rosƵ�ʶ���rate
		while (ros::ok()) {
			rate.sleep();//�ȴ��ԴﵽƵ��
			ros::spinOnce();//�������еĻص�����
			if (front_end_ptr->track()) {//���ʱ���ͳһ�򷢲�
				publishMsg();
			}
		}
	}

	void FrontEndWrapper::publishMsg() {
		static nav_msgs::Path path;
		auto X = front_end_ptr->readState();//ת��Ϊ��������ϵ�ĵ���
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
