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
            Eigen::Quaterniond rotation;//旋转
            Eigen::Vector3d position;//位移
            Eigen::Vector3d velocity;//速度
            Eigen::Vector3d bg;//重力偏移
            Eigen::Vector3d ba;//加速度偏移
            Eigen::Vector3d gravity;//重力
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
        Eigen::Matrix<double, 18, 18> P;//状态协方差矩阵
        Eigen::Matrix<double, 12, 12> Q;//过程噪声协方差矩阵
        int iter_times = 10;

    public:
        IESKF(const std::string& config_path, const std::string& prefix);
        ~IESKF();
        void predict(IMU imu, double dt);
        bool update();
        //获取系统状态X
        const State18& getX();
        void setX(const State18& x_in);
        //计算两个状态的差异
        Eigen::Matrix<double, 18, 1> getErrorState18(const State18& s1, const State18& s2);
    };


}