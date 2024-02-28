#include "include/slam/modules/frontend/frontend.h"
#include "pcl/common/transforms.h"

namespace Slam
{
    //在构造FrontEnd对象时，同时构造其基类ModuleBase对象，完成基类初始化
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
    //对齐pose和point的时间戳
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
        //体素滤波
        VoxelFilter vf;
        vf.setLeafSize(0.5, 0.5, 0.5);
        vf.setInputCloud(pointcloud_deque.front().cloud_ptr);
        vf.filter(*pointcloud_deque.front().cloud_ptr);
        
        //求得变换矩阵
        Eigen::Matrix4f trans;//创建4x4矩阵，表示变换矩阵
        trans.setIdentity();//设为单位阵
        //将姿态信息中的旋转部分转换为旋转矩阵，并赋值给变换矩阵的旋转部分。
        //pose_deque.front().rotation.toRotationMatrix() 返回的是姿态信息中的旋转矩阵，
        //.cast<float>() 将其转换为浮点数类型。
        trans.block<3, 3>(0, 0) = pose_deque.front().rotation.toRotationMatrix().cast<float>();
        trans.block<3, 1>(0, 3) = pose_deque.front().position.cast<float>();

        //坐标变换，存储在current_pointcloud
        pcl::transformPointCloud(*pointcloud_deque.front().cloud_ptr, current_pointcloud, trans);
        pose_deque.pop_front();
        pointcloud_deque.pop_front();
        return true;
    }

    const PCLPointCloud& FrontEnd::readCurrentPointCloud() {
        return current_pointcloud;
    }
}