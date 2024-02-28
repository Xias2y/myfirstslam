class ModuleBase
{
private:
	YAML::Node config_node;  //yaml节点对象，map、seq、scalars
	std::string name;
protected:
	/**
	 * @param config_path: 配置文件目录
	 * @param prefix: 前缀
	 * @param module_name: 模块名称
	*/
	ModuleBase(const std::string& config_path, const std::string& prefix, const std::string& module_name = "default") {
		name = module_name;
		if (config_path != "") {
			try {
				config_node = YAML::LoadFile(config_path);
			}
			catch (YAML::Exception &e) {
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
	void readParam(const std::string &key, T &val, T default_val) {
		if (config_node[key]) {
			val = config_node[key].as<T>();  //as<T>是模板转换（取值时用）
		}
		else {
			val = default_val;
		}
	}
	//构造函数：模块继承,只需要在构造的时候传入配置文件路径即可完成配置的加载
	//模板函数：通过readParam读取参数
};