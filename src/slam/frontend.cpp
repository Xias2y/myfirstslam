#include "include/slam/modules/frontend/frontend.h"
#include "pcl/common/transforms.h"

namespace Slam
{
    //�ڹ���FrontEnd����ʱ��ͬʱ���������ModuleBase������ɻ����ʼ��
    FrontEnd::FrontEnd(const std::string& config_file_path, const std::string& prefix) :ModuleBase(config_file_path, prefix, "Front End Module")
    {

    }

    FrontEnd::~FrontEnd()
    {
    }
    void FrontEnd::addImu(const IMU&imu) {
        imu_deque.push_back(imu);
    }
    void FrontEnd::addPointCloud(const PointCloud& pointcloud) {
        pointcloud_deque.push_back(pointcloud);
    }
    void FrontEnd::addPose(const Pose& pose); {
        pose_deque.push_back(pose);
    }
    //����pose��point��ʱ���
    bool FrontEnd::track() {
        if (pose_deque.empty() || pointcloud_deque.empty()) {
            return false;
        }
        while (!pose_deque.empty() && pose_deque.front().time_stamp.nsec() < pointcloud_deque.front().time_stamp.nsec())
        {
            std::cout << "pop_pose" << std::end1;
            pose_deque.pop_front();
        }
        if (pose_deque.empty()){
            return false;
        }
        while (!pointcloud_deque.empty() && pointcloud_deque.front().time_stamp.nsec() < pose_deque.front().time_stamp.nsec())
        {
            std::cout << "pop_point" << std::endl;
            pointcloud_deque.pop_front();
        }
        if (pointcloud_deque.empty()) {
            return false;
        }
        //�����˲�
        VoxelFilter vf;
        vf.setLeafSize(0.5, 0.5, 0.5);
        vf.setInputCloud(pointcloud_deque.front().cloud_ptr);
        vf.filter(*pointcloud_deque.front().cloud_ptr);
        
        //��ñ任����
        Eigen::Matrix4f trans;//����4x4���󣬱�ʾ�任����
        trans.setIdentity();//��Ϊ��λ��
        //����̬��Ϣ�е���ת����ת��Ϊ��ת���󣬲���ֵ���任�������ת���֡�
        //pose_deque.front().rotation.toRotationMatrix() ���ص�����̬��Ϣ�е���ת����
        //.cast<float>() ����ת��Ϊ���������͡�
        trans.block<3, 3>(0, 0) = pose_deque.front().rotation.toRotationMatrix().cast<float>();
        trans.block<3, 1>(0, 3) = pose_deque.front().position.cast<float>();

        //����任���洢��current_pointcloud
        pcl::transformPointCloud(*pointcloud_deque.front().cloud_ptr, current_pointcloud, trans);
        pose_deque.pop_front();
        pointcloud_deque.pop_front();
        return true;
    }

    const PCLPointCloud& FrontEnd::readCurrentPointCloud() {
        return current_pointcloud;
    }
}