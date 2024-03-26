#pragma once
#include "slam/modules/module_base.h"
#include <Eigen/Dense>
#include "slam/type/imu.h"
#include "slam/math/SO3.h"

namespace Slam
{
    class IESKF:private ModuleBase
    {
    public:
        using Ptr = std::shared_ptr<IESKF>;
        struct State18
        {
            Eigen::Quaterniond rotation;//��ת
            Eigen::Vector3d position;//λ��
            Eigen::Vector3d velocity;//�ٶ�
            Eigen::Vector3d bg;//����ƫ��
            Eigen::Vector3d ba;//���ٶ�ƫ��
            Eigen::Vector3d gravity;//����
            State18() {
                rotation = Eigen::Quaterniond::Identity();
                position = Eigen::Vector3d::Zero();
                velocity = Eigen::Vector3d::Zero();
                bg = Eigen::Vector3d::Zero();
                ba = Eigen::Vector3d::Zero();
                gravity = Eigen::Vector3d::Zero();
            }
        };

        class CalcZHInterface
        {
        public:
            virtual bool calculate(const State18& state, Eigen::MatrixXd& Z, Eigen::MatrixXd& H) = 0;
        };
        std::shared_ptr<CalcZHInterface> calc_zh_ptr;

    private:
        State18 X;
        Eigen::Matrix<double, 18, 18> P;//״̬Э�������
        Eigen::Matrix<double, 12, 12> Q;//��������Э�������
        int iter_times = 10;

    public:
        IESKF(const std::string& config_path, const std::string& prefix);
        ~IESKF();
        void predict(IMU imu, double dt);
        bool update();
        //��ȡϵͳ״̬X
        const State18& getX();
        void setX(const State18& x_in);
        //��������״̬�Ĳ���
        Eigen::Matrix<double, 18, 1> getErrorState18(const State18& s1, const State18& s2);
    };


}