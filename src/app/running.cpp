#include "wrapper/frontend_wrapper.h"

int main(int argc, char* argv[])//ros��ʼ��
{
	ros::init(argc, argv, "running");//��ʼ��running�ڵ� 
	ros::NodeHandle nh;
	//������һ������ָ�룬ָ��FrontEndWrapper
	std::shared_ptr<ROSNoetic::FrontEndWrapper>front_end_ptr;
	//������һ��ֻ��ָ����󣬳�ʼ��ָ��FrontEndWrapper
	front_end_ptr = std::make_shared<ROSNoetic::FrontEndWrapper>(nh);
	return 0;
}