#include "slam/modules/frontbackPropagate/frontback_propagate.h"
#include "slam/type/point.h"

namespace Slam
{

    FrontbackPropagate::FrontbackPropagate(/* args */)
    {
    }

    FrontbackPropagate::~FrontbackPropagate()
    {
    }

    //  先递推imu，然后记录下递推后的结果，完成递推的结果后，再对每个点进行去畸变
    void FrontbackPropagate::propagate(MeasureGroup& mg, IESKF::Ptr ieskf_ptr) {
        std::sort(mg.cloud.cloud_ptr->points.begin(), mg.cloud.cloud_ptr->points.end(), 
            [](Point x, Point y) {return x.offset_time < y.offset_time;});
        //  记录每一步递推的结果
        std::vector<IMUPose6d> IMUpose;
        auto v_imu = mg.imus;
        v_imu.push_front(last_imu_);
        const double& imu_beg_time = v_imu.front().time_stamp.sec();//  imu数据获取的时刻
        const double& imu_end_time = v_imu.back().time_stamp.sec();
        const double& pcl_beg_time = mg.lidar_begin_time;//  点数据获取的时刻
        const double& pcl_end_time = mg.lidar_end_time;
        auto& pcl_out = *mg.cloud.cloud_ptr;
        auto imu_state = ieskf_ptr->getX();
        IMUpose.clear();
        //  记录递推起始时刻的状态
        IMUpose.emplace_back(0.0, acc_s_last, angvel_last, imu_state.velocity, imu_state.position, 
                            imu_state.rotation);//  尾部创建新元素
        Eigen::Vector3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
        Eigen::Matrix3d R_imu;
        double dt = 0;
        IMU in;
        //  开始递推
        for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
        {
            //  取两帧imu
            auto&& head = *(it_imu);
            auto&& tail = *(it_imu + 1);
            if (tail.time_stamp.sec() < last_lidar_end_time_)
                continue;
            //  求平均角速度、加速度
            angvel_avr = 0.5 * (head.gyroscope + tail.gyroscope);
            acc_avr = 0.5 * (head.acceleration + tail.acceleration);
            //  统一单位到m/s^2
            acc_avr = acc_avr * imu_scale;
            if (head.time_stamp.sec() < last_lidar_end_time_)
            {
                dt = tail.time_stamp.sec() - last_lidar_end_time_;
            }
            else
            {
                dt = tail.time_stamp.sec() - head.time_stamp.sec();
            }
            in.acceleration = acc_avr;
            in.gyroscope = angvel_avr;
            //  预测步
            ieskf_ptr->predict(in, dt);
            //  一帧扫描终点的位姿
            imu_state = ieskf_ptr->getX();
            //  计算这个时刻的lidar系的速度和位姿，为后向传播做准备
            angvel_last = angvel_avr - imu_state.bg;
            acc_s_last = imu_state.rotation * (acc_avr - imu_state.ba);
            //  消除加速度中的重力分量
            for (int i = 0; i < 3; i++)
            {
                acc_s_last[i] += imu_state.gravity[i];
            }
            double&& offs_t = tail.time_stamp.sec() - pcl_beg_time;
            //  存储
            IMUpose.emplace_back(offs_t, acc_s_last, angvel_last, imu_state.velocity,
                                imu_state.position, imu_state.rotation);
        }
        //  imu频率大于雷达，一帧点云以多出来一部分imu结束，将雷达位姿预测到imu的结束时刻
        dt = pcl_end_time - imu_end_time;
        ieskf_ptr->predict(in, dt);
        // 此时的递推结果就是雷达末尾的位姿，即上图中C的位置
        imu_state = ieskf_ptr->getX();
        last_imu_ = mg.imus.back();
        last_lidar_end_time_ = pcl_end_time;
        if (pcl_out.points.begin() == pcl_out.points.end())
            return;
        auto it_pcl = pcl_out.points.end() - 1;
        // 从后往前对每个点去畸变，这里开始就是后向传播部分
        // 先从存储的递推姿态中从后往前取状态信息，然后把这个时间段内的点云去畸变
        for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
        {
            // T_k-1的位姿 // 可以当作上图中A时刻的位姿状态
            auto head = it_kp - 1;
            auto tail = it_kp;
            R_imu = head->rot.toRotationMatrix();
            vel_imu = head->vel;
            pos_imu = head->pos;
            acc_imu = tail->acc;
            angvel_avr = tail->angvel;
            for (; it_pcl->offset_time / 1e9 > head->time; it_pcl--)
            {
                //计算 T_i，也就是下面的R_i和T_ei i处于k-1至k之间，i为点的时间戳。
                dt = it_pcl->offset_time / 1e9 - head->time;
                Eigen::Matrix3d R_i(R_imu * so3Exp(angvel_avr * dt));
                Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
                // 这个变量不是SE(3),而是一个位置偏移。T_ei = t^i -t^C,t_i是 i时刻雷达系的位姿，t^C是扫描末尾的雷达位姿
                // 我这里没有自己写，照抄了一部分fast-lio的代码
                Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - 
                                    imu_state.position);
                // 坐标系转换 P = (T_C^G)^{-1}*T_B^G*P^B
                Eigen::Vector3d P_compensate = imu_state.rotation.conjugate() * (R_i * P_i + T_ei);
                // 重新赋值
                it_pcl->x = P_compensate(0);
                it_pcl->y = P_compensate(1);
                it_pcl->z = P_compensate(2);
                if (it_pcl == pcl_out.points.begin())
                    break;
            }
        }
        return;
    }
}