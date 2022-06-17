#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <pcl_ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  

  tf::TransformListener listener;  

 
  ros::Rate rate(10.0);
  while (node.ok()){
    //创建一个StampedTransform对象存储变换结果数据
    tf::StampedTransform transform_lidar;
    tf::StampedTransform transform_cam;
    
    //监听包装在一个try-catch块中以捕获可能的异常
    try{

      listener.lookupTransform("/rslidar", "/rotated_camera",
                               ros::Time(0), transform_lidar);
      listener.lookupTransform("/rotated_camera", "/camera",
                               ros::Time(0), transform_cam);
      Eigen::Matrix4f lidar2cam;
      Eigen::Matrix4f cam2cam;
      pcl_ros::transformAsMatrix(transform_lidar, lidar2cam);   //直接得到矩阵
      pcl_ros::transformAsMatrix(transform_cam, cam2cam);   //直接得到矩阵
      std::cout << "lidar2cam:"  <<std::endl;
      std::cout << lidar2cam  <<std::endl;
      std::cout << "cam2cam:"  <<std::endl;
      std::cout << cam2cam  <<std::endl;
      std::cout << "final:"  <<std::endl;
      std::cout << lidar2cam*cam2cam  <<std::endl;
      std::cout << "final2:"  <<std::endl;
      // std::cout << (lidar2cam*cam2cam).inverse()  <<std::endl;
      std::cout << (cam2cam*lidar2cam).inverse()  <<std::endl;

    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }
  return 0;
};


