#pragma once
#include "slam/modules/ieskf/ieskf.h"
#include "slam/type/base_type.h"
#include "slam/math/geometry.h"

namespace Slam
{
	class LIOZHModel :public IESKF::CalcZHInterface
	{
	public:
		using Ptr = std::shared_ptr<LIOZHModel>;
		void prepare(KDTreeConstPtr kd_tree, PCLPointCloudPtr current_cloud, PCLPointCloudConstPtr local_map) {
			global_map_kdtree_ptr = kd_tree;
			current_cloud_ptr = current_cloud;
			local_map_ptr = local_map;
		}
		bool calculate(const IESKF::State18& state, Eigen::MatrixXd& Z, Eigen::MatrixXd& H)override {
			std::vector<loss_type> loss_v;
			loss_v.resize(current_cloud_ptr->size());//重设大小
			std::vector<bool> is_effect_point(current_cloud_ptr->size(), false);
			std::vector<loss_type> loss_real;
			int  vaild_points_num = 0;

#ifdef MP_EN //根据MP_EN的定义来决定是否使用多线程计算
			omp_set_num_threads(MP_PROC_NUM);//设置线程数量
#pragma omp parallel for//多线程运行for循环
#endif
			/**
			 有效点的判断
			 1. 将当前点变换到世界系下
			 2. 在局部地图上使用kd_tree 临近搜索NEAR_POINTS_NUM个点
			 3. 判断这些点是否构成平面
			 4. 判断点离这个平面够不够近(达到阈值)
			 5. 满足上述条件，设置为有效点。
			*/
			for (size_t i = 0; i < current_cloud_ptr->size(); i++)
			{
				//1.变换到世界系
				Point point_imu = current_cloud_ptr->points[i];
				Point point_world;
				point_world = transformPoint(point_imu, state.rotation, state.position);
				//2.临近搜索
				std::vector<int> point_ind;//索引
				std::vector<float> distance;//距离
				global_map_kdtree_ptr->nearestKSearch(point_world, NEAR_POINTS_NUM, point_ind, distance);//nearestKSearch是PCL中KDtree最近邻搜索
				//是否搜索到足够的点以及最远的点到当前点的距离足够小(太远，就不认为这俩在一个平面)
				if (distance.size() < NEAR_POINTS_NUM || distance[NEAR_POINTS_NUM - 1] > 5)
				{
					continue;
				}
				//3.判断这些点够不够成平面
				std::vector<Point> planar_points;
				for (int ni = 0; ni < NEAR_POINTS_NUM; ni++)
				{
					planar_points.push_back(local_map_ptr->at(point_ind[ni]));
				}
				Eigen::Vector4d pabcd;
				//如果构成平面
				if (planarCheck(planar_points, pabcd, 0.1))
				{
					//计算点到平面距离
					double pd = point_world.x * pabcd(0) + point_world.y * pabcd(1) + point_world.z * pabcd(2) + pabcd(3);
					//记录残差
					loss_type loss;
					loss.thrid = pd; // 残差
					loss.first = { point_imu.x,point_imu.y,point_imu.z }; // imu系下点的坐标，用于求H
					loss.second = pabcd.block<3, 1>(0, 0);// 平面法向量 用于求H
					if (isnan(pd) || isnan(loss.second(0)) || isnan(loss.second(1)) || isnan(loss.second(2)))continue;
					//计算点和平面的夹角，夹角越小S越大。
					double s = 1 - 0.9 * fabs(pd) / sqrt(loss.first.norm());
					if (s > 0.9) {

						vaild_points_num++;
						loss_v[i] = (loss);
						is_effect_point[i] = true;
					}
				}
			}
			for (size_t i = 0; i < current_cloud_ptr->size(); i++)
			{
				if (is_effect_point[i])loss_real.push_back(loss_v[i]);
			}
			// 根据有效点的数量分配H Z的大小
			vaild_points_num = loss_real.size();
			H = Eigen::MatrixXd::Zero(vaild_points_num, 18);
			Z.resize(vaild_points_num, 1);
			for (int vi = 0; vi < vaild_points_num; vi++)
			{
				// H 记录导数
				Eigen::Vector3d dr = -1 * loss_real[vi].second.transpose() * state.rotation.toRotationMatrix() * skewSymmetric(loss_real[vi].first);
				H.block<1, 3>(vi, 0) = dr.transpose();
				H.block<1, 3>(vi, 3) = loss_real[vi].second.transpose();
				// Z记录距离
				Z(vi, 0) = loss_real[vi].thrid;
			}
			return true;
		}
	}；

	private:
		const int NEAR_POINTS_NUM = 5;//临近点数量
		//坐标、法向量、距离     
		using loss_type = triple<Eigen::Vector3d, Eigen::Vector3d, double>;
		KDTreeConstPtr global_map_kdtree_ptr;
		PCLPointCloudPtr current_cloud_ptr;
		PCLPointCloudConstPtr local_map_ptr;
	};
}