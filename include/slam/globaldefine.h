#pragma once
#include <string>

#ifndef PROJ_DIR
#define PROJ_DIR " "
#endif

//定义工作目录和配置目录路径
const std::string WORKD_DIR = PROJ_DIR;
const std::string CONFIG_DIR = WORKD_DIR + "/config/";
const std::string RESULT_DIR = WORKD_DIR + "/result/";