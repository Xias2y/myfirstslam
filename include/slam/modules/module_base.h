#pragma once
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "colorful_terminal/colorful_terminal.hpp"

namespace Slam{
	class ModuleBase
	{
	private:
		YAML::Node config_node;  //yaml�ڵ����map��seq��scalars
		std::string name;
		std::shared_ptr<ctl::table_out> table_out_ptr;
	protected:
		/**
		 * @param config_path: �����ļ�Ŀ¼
		 * @param prefix: ǰ׺
		 * @param module_name: ģ������
		*/
		ModuleBase(const std::string& config_path, const std::string& prefix, const std::string& module_name = "default") {
			name = module_name;
			//  ����ն�
			table_out_ptr = std::make_shared<ctl::table_out>(module_name);
			if (config_path != "") {
				try {
					config_node = YAML::LoadFile(config_path);
				}
				catch (YAML::Exception& e) {
					std::cout << e.msg << std::endl;
				}
				if (prefix != "" && config_node[prefix]) config_node = config_node[prefix];
			}
		}
		/**
		 * @param T
		 * @param key: ��ֵ
		 * @param val: ��ȡ���ݵ��ĸ�����
		 * @param default_val: Ĭ��ֵ
		*/
		template<typename T>
		void readParam(const std::string& key, T& val, T default_val) {
			if (config_node[key]) {
				val = config_node[key].as<T>();  //as<T>��ģ��ת����ȡֵʱ�ã�
			}
			else {
				val = default_val;
			}
			table_out_ptr->add_item(key, VAR_NAME(val), val);
		}
		void print_table() { table_out_ptr->make_table_and_out(); }
	};
		//���캯����ģ��̳�,ֻ��Ҫ�ڹ����ʱ���������ļ�·������������õļ���
		//ģ�庯����ͨ��readParam��ȡ����
}
