#include "wrapper/frontend_wrapper.h"

int main(int argc, char* argv[])//ros初始化
{
	ros::init(argc, argv, "running");//初始化running节点 
	ros::NodeHandle nh;
	//定义了一个智能指针，指向FrontEndWrapper
	std::shared_ptr<ROSNoetic::FrontEndWrapper>front_end_ptr;
	//创建了一个只能指针对象，初始化指向FrontEndWrapper
	front_end_ptr = std::make_shared<ROSNoetic::FrontEndWrapper>(nh);
	return 0;
}