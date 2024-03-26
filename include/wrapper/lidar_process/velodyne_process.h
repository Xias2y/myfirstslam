#pragma once
#include "wrapper/lidar_process/common_lidar_process_interface.h"

namespace velodyne_ros {
	struct EIGEN_ALIGN16 Point
	{
		PCL_ADD_POINT4D;
		float intensity;
		uint16_t ring;
		float time;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, intensity, intensity)
	(float, time, time)
	(std::uint16_t, ring, ring)
	)

    namespace ROSNoetic
{
    class VelodyneProcess :public CommonLidarProcessInterface
    {
    private:
    public:
        bool process(const sensor_msgs::PointCloud2& msg, Slam::PointCloud& cloud) {
            pcl::PointCloud<velodyne_ros::Point> rs_cloud;
            pcl::fromROSMsg(msg, rs_cloud);
            cloud.cloud_ptr->clear();
            double end_time = msg.header.stamp.toSec();
            //  ������ʼʱ�� ʱ���+��0�����ƫ��
            double start_time = end_time + rs_cloud[0].time;
            for (auto&& p : rs_cloud)
            {
                double point_time = p.time + end_time;
                Slam::Point point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;
                point.intensity = p.intensity;


                point.offset_time = (point_time - start_time) * 1e9;
                point.ring = p.ring;
                cloud.cloud_ptr->push_back(point);

            }
            cloud.time_stamp.fromSec(start_time);
            return true;
        }
    };
}