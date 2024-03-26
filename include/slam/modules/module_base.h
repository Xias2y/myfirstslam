#pragma once
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "colorful_terminal/colorful_terminal.hpp"

namespace Slam{
	class ModuleBase
	{
	private:
		YAML::Node config_node;  //yaml节点对象，map、seq、scalars
		std::string name;
		std::shared_ptr<ctl::table_out> table_out_ptr;
	protected:
		/**
		 * @param config_path: 配置文件目录
		 * @param prefix: 前缀
		 * @param module_name: 模块名称
		*/
		ModuleBase(const std::string& config_path, const std::string& prefix, const std::string& module_name = "default") {
			name = module_name;
			//  输出终端
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
		 * @param key: 键值
		 * @param val: 读取数据到哪个参数
		 * @param default_val: 默认值
		*/
		template<typename T>
		void readParam(const std::string& key, T& val, T default_val) {
			if (config_node[key]) {
				val = config_node[key].as<T>();  //as<T>是模板转换（取值时用）
			}
			else {
				val = default_val;
			}
			table_out_ptr->add_item(key, VAR_NAME(val), val);
		}
		void print_table() { table_out_ptr->make_table_and_out(); }
	};
		//构造函数：模块继承,只需要在构造的时候传入配置文件路径即可完成配置的加载
		//模板函数：通过readParam读取参数
}
