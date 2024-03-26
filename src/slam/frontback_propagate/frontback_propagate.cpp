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

    //  �ȵ���imu��Ȼ���¼�µ��ƺ�Ľ������ɵ��ƵĽ�����ٶ�ÿ�������ȥ����
    void FrontbackPropagate::propagate(MeasureGroup& mg, IESKF::Ptr ieskf_ptr) {
        std::sort(mg.cloud.cloud_ptr->points.begin(), mg.cloud.cloud_ptr->points.end(), 
            [](Point x, Point y) {return x.offset_time < y.offset_time;});
        //  ��¼ÿһ�����ƵĽ��
        std::vector<IMUPose6d> IMUpose;
        auto v_imu = mg.imus;
        v_imu.push_front(last_imu_);
        const double& imu_beg_time = v_imu.front().time_stamp.sec();//  imu���ݻ�ȡ��ʱ��
        const double& imu_end_time = v_imu.back().time_stamp.sec();
        const double& pcl_beg_time = mg.lidar_begin_time;//  �����ݻ�ȡ��ʱ��
        const double& pcl_end_time = mg.lidar_end_time;
        auto& pcl_out = *mg.cloud.cloud_ptr;
        auto imu_state = ieskf_ptr->getX();
        IMUpose.clear();
        //  ��¼������ʼʱ�̵�״̬
        IMUpose.emplace_back(0.0, acc_s_last, angvel_last, imu_state.velocity, imu_state.position, 
                            imu_state.rotation);//  β��������Ԫ��
        Eigen::Vector3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
        Eigen::Matrix3d R_imu;
        double dt = 0;
        IMU in;
        //  ��ʼ����
        for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
        {
            //  ȡ��֡imu
            auto&& head = *(it_imu);
            auto&& tail = *(it_imu + 1);
            if (tail.time_stamp.sec() < last_lidar_end_time_)
                continue;
            //  ��ƽ�����ٶȡ����ٶ�
            angvel_avr = 0.5 * (head.gyroscope + tail.gyroscope);
            acc_avr = 0.5 * (head.acceleration + tail.acceleration);
            //  ͳһ��λ��m/s^2
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
            //  Ԥ�ⲽ
            ieskf_ptr->predict(in, dt);
            //  һ֡ɨ���յ��λ��
            imu_state = ieskf_ptr->getX();
            //  �������ʱ�̵�lidarϵ���ٶȺ�λ�ˣ�Ϊ���򴫲���׼��
            angvel_last = angvel_avr - imu_state.bg;
            acc_s_last = imu_state.rotation * (acc_avr - imu_state.ba);
            //  �������ٶ��е���������
            for (int i = 0; i < 3; i++)
            {
                acc_s_last[i] += imu_state.gravity[i];
            }
            double&& offs_t = tail.time_stamp.sec() - pcl_beg_time;
            //  �洢
            IMUpose.emplace_back(offs_t, acc_s_last, angvel_last, imu_state.velocity,
                                imu_state.position, imu_state.rotation);
        }
        //  imuƵ�ʴ����״һ֡�����Զ����һ����imu���������״�λ��Ԥ�⵽imu�Ľ���ʱ��
        dt = pcl_end_time - imu_end_time;
        ieskf_ptr->predict(in, dt);
        // ��ʱ�ĵ��ƽ�������״�ĩβ��λ�ˣ�����ͼ��C��λ��
        imu_state = ieskf_ptr->getX();
        last_imu_ = mg.imus.back();
        last_lidar_end_time_ = pcl_end_time;
        if (pcl_out.points.begin() == pcl_out.points.end())
            return;
        auto it_pcl = pcl_out.points.end() - 1;
        // �Ӻ���ǰ��ÿ����ȥ���䣬���￪ʼ���Ǻ��򴫲�����
        // �ȴӴ洢�ĵ�����̬�дӺ���ǰȡ״̬��Ϣ��Ȼ������ʱ����ڵĵ���ȥ����
        for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
        {
            // T_k-1��λ�� // ���Ե�����ͼ��Aʱ�̵�λ��״̬
            auto head = it_kp - 1;
            auto tail = it_kp;
            R_imu = head->rot.toRotationMatrix();
            vel_imu = head->vel;
            pos_imu = head->pos;
            acc_imu = tail->acc;
            angvel_avr = tail->angvel;
            for (; it_pcl->offset_time / 1e9 > head->time; it_pcl--)
            {
                //���� T_i��Ҳ���������R_i��T_ei i����k-1��k֮�䣬iΪ���ʱ�����
                dt = it_pcl->offset_time / 1e9 - head->time;
                Eigen::Matrix3d R_i(R_imu * so3Exp(angvel_avr * dt));
                Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
                // �����������SE(3),����һ��λ��ƫ�ơ�T_ei = t^i -t^C,t_i�� iʱ���״�ϵ��λ�ˣ�t^C��ɨ��ĩβ���״�λ��
                // ������û���Լ�д���ճ���һ����fast-lio�Ĵ���
                Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - 
                                    imu_state.position);
                // ����ϵת�� P = (T_C^G)^{-1}*T_B^G*P^B
                Eigen::Vector3d P_compensate = imu_state.rotation.conjugate() * (R_i * P_i + T_ei);
                // ���¸�ֵ
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