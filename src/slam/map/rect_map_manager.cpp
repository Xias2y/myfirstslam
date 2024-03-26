#include "slam/modules/map/rect_map_manager.h"

namespace Slam
{
    RectMapManager::RectMapManager(const std::string& config_file_path, const std::string& prefix) 
        :ModuleBase(config_file_path, prefix, "RectMapManager")
    {
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
        kdtree_ptr = pcl::make_shared<KDTree>();
        readParam<float>("map_side_length_2", map_side_length_2, 500);
        readParam<float>("map_resolution", map_resolution, 0.5);
        print_table();
    }

    RectMapManager::~RectMapManager()
    {
    }
    //�����ƣ���̬��λ�ã�
    //��ǰֱ֡�Ӽ���ֲ���ͼ+�²���
    void RectMapManager::addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond& att_q, 
                                const Eigen::Vector3d& pos_t) {
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan, scan, compositeTransform(att_q, pos_t).cast<float>());
        if (local_map_ptr->empty()) {
            *local_map_ptr = scan;//��ǰ֡����ֲ���ͼ
        }
        else
        {
            for (auto&& point : scan)
            {
                std::vector<int> ind;
                std::vector<float> distance;
                kdtree_ptr->nearestKSearch(point, 5, ind, distance);
                if (distance[0] > map_resolution) local_map_ptr->push_back(point);
            }
            int left = 0, right = local_map_ptr->size() - 1;
            while (left < right) {
                while (left < right &&
                    abs(local_map_ptr->points[right].x - pos_t.x()) > map_side_length_2 ||
                    abs(local_map_ptr->points[right].y - pos_t.y()) > map_side_length_2 ||
                    abs(local_map_ptr->points[right].z - pos_t.z()) > map_side_length_2)
                    right--;
                while (left < right &&
                    abs(local_map_ptr->points[left].x - pos_t.x()) < map_side_length_2 &&
                    abs(local_map_ptr->points[left].y - pos_t.y()) < map_side_length_2 &&
                    abs(local_map_ptr->points[left].z - pos_t.z()) < map_side_length_2)
                    left++;
                std::swap(local_map_ptr->points[left], local_map_ptr->points[right]);
            }
            local_map_ptr->resize(right + 1);//right�Ǿֲ���ͼ�����һ��Ҫ�����ĵ������
        }
        kdtree_ptr->setInputCloud(local_map_ptr);
    }
    void RectMapManager::reset() {
        local_map_ptr->clear();
    }
    PCLPointCloudConstPtr RectMapManager::getLocalMap() {
        return local_map_ptr;
    }

    KDTreeConstPtr RectMapManager::readKDtree()
    {
        return kdtree_ptr;
    }
}