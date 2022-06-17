#include "ros/ros.h"

int main(int argc, char  *argv[])
{
    /* code */
    //解决中文乱码问题
    setlocale(LC_ALL,"");
    
    ros::init(argc,argv,"hello_c");
    ROS_INFO("hello,哈哈哈");
    
    return 0;
}
