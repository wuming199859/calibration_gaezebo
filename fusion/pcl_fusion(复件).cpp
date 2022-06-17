#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "myPointType.h"

cv::Vec3b  image_color[480][640];  //全局变量都能访问，图像中修改，点云中获取

// cv::Mat intrisicMat(3, 3, cv::DataType<double>::type); // Intrisic matrix 内参矩阵
// cv::Mat rVec(3, 1, cv::DataType<double>::type);// Rotation vector 旋转矩阵
// cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector  平移矩阵
// cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector 畸变向量

cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification 矫正后的3*4的投影矩阵
cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector 旋转矩阵和平移向量

class pcl_fusion
{
private:
  ros::NodeHandle n;
  sensor_msgs::PointCloud2 msg;  //接收到的点云消息
  sensor_msgs::PointCloud2 fusion_msg;  //等待发送的点云消息
  

public:
  ros::Subscriber subCloud = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &pcl_fusion::getcloud, this); //接收velodyne点云数据，进入回调函数getcloud()
  ros::Publisher pubCloud = n.advertise<sensor_msgs::PointCloud2>("/fusion_cloud", 1);  //建立了一个发布器，主题是/adjustd_cloud，方便之后发布加入颜色之后的点云
  
  //点云回调函数
  void getcloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    pcl::PointCloud<PointXYZRGBIR>::Ptr  fusion_pcl_ptr (new pcl::PointCloud<PointXYZRGBIR>);   //放在这里是因为，每次都需要重新初始化，舍弃了原有但没用的两个通道intensity、ring
    pcl::PointCloud<PointXYZIR>::Ptr   raw_pcl_ptr (new pcl::PointCloud<PointXYZIR>);   //VLP-16的点云消息包含xyz和intensity、ring的，这里没有加ring不知道为啥也可以，需要的话要自己定义一个点类型PointXYZIR
    pcl::fromROSMsg(*laserCloudMsg, *raw_pcl_ptr);  //把msg消息指针转化为点云指正


    // 第一种做法
    // std::vector<cv::Point3d> objectPoints;
    // std::vector<cv::Point2d> imagePoints;
    // uint8_t row, col;
    // for(int i=0;i<=raw_pcl_ptr->points.size();i++)
    // {
    //     objectPoints.push_back(cv::Point3d(raw_pcl_ptr->points[i].x,raw_pcl_ptr->points[i].y, raw_pcl_ptr->points[i].z));
    // }
    // cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, imagePoints);

    // for (int i = 0; i <  raw_pcl_ptr->points.size(); i++)
    // {
    //   row = round(imagePoints[i].x);
    //   col = round(imagePoints[i].y);
    //   // row = 200;
    //   // col = 100;
    //   if ( col >=0 && col <  640 && row>=0 && row<480 )
    //   {
    //     PointXYZRGBIR  p;
    //     p.x=raw_pcl_ptr->points[i].x;
    //     p.y=raw_pcl_ptr->points[i].y;
    //     p.z=raw_pcl_ptr->points[i].z;
    //     //点云颜色由图像上对应点确定
    //     p.b = image_color[row][col][0];
    //     p.g = image_color[row][col][1];
    //     p.r = image_color[row][col][2];

    //     p.label = (rand() % (10+1));  //设置10个标签，标签随机
    //     p.intensity = raw_pcl_ptr->points[i].intensity;  //继承之前点云的intensity
    //     p.ring = raw_pcl_ptr->points[i].ring;  //继承之前点云的ring
    //     fusion_pcl_ptr->points.push_back(p);
    //   }
    // }


    //另一种做法
    cv::Mat X(4,1,cv::DataType<double>::type);
    cv::Mat Y(3,1,cv::DataType<double>::type);   
    for (int i = 0; i <  raw_pcl_ptr->points.size(); i++)
    {
      X.at<double>(0,0) = raw_pcl_ptr->points[i].x;
      X.at<double>(1,0) = raw_pcl_ptr->points[i].y;
      X.at<double>(2,0) = raw_pcl_ptr->points[i].z;
      X.at<double>(3,0) = 1;
      Y = P_rect_00 * RT * X;  //坐标转换
      cv::Point pt;
      pt.x =  Y.at<double>(0,0) / Y.at<double>(0,2) ;
      pt.y = Y.at<double>(1,0) / Y.at<double>(0,2) ;
      // std::cout<<  pt << std::endl;

      if ( pt.x >=0 &&pt.x <  640 && pt.y>=0 && pt.y<480 && raw_pcl_ptr->points[i].x>0) //&& raw_pcl_ptr->points[i].x>0去掉图像后方的点云
      {
        PointXYZRGBIR  p;
        p.x=raw_pcl_ptr->points[i].x;
        p.y=raw_pcl_ptr->points[i].y;
        p.z=raw_pcl_ptr->points[i].z;
        //点云颜色由图像上对应点确定
        p.b = image_color[pt.y][pt.x][0];
        p.g = image_color[pt.y][pt.x][1];
        p.r = image_color[pt.y][pt.x][2];

        p.label = (rand() % (10+1));  //设置10个标签，标签随机
        p.intensity = raw_pcl_ptr->points[i].intensity;  //继承之前点云的intensity
        p.ring = raw_pcl_ptr->points[i].ring;  //继承之前点云的ring
        fusion_pcl_ptr->points.push_back(p);
      }
    }

    fusion_pcl_ptr->width = 1;
    fusion_pcl_ptr->height = fusion_pcl_ptr->points.size();
    // std::cout<<  fusion_pcl_ptr->points.size() << std::endl;
    pcl::toROSMsg( *fusion_pcl_ptr, fusion_msg);  //将点云转化为消息才能发布
    fusion_msg.header.frame_id = "velodyne";//帧id改成和velodyne一样的
    pubCloud.publish( fusion_msg); //发布调整之后的点云数据，主题为/adjustd_cloud
  }

};

 //图像回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv::Mat image  = cv_bridge::toCvShare(msg, "bgr8")->image; //image_raw就是我们得到的图像了
    // cv::circle(image,cv::Point(100,250),5,cv::Scalar(0,0,255),3); //注意先列后行
    for (int row = 0; row < 480; row++ )
    {
      for (int  col= 0; col< 640; col++ )
      {
        image_color[row][col] = (cv::Vec3b)image.at<cv::Vec3b>(row, col);
      }
    }
    // cv::imshow("view", image);
  }
  catch (cv_bridge::Exception& e){
  ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void loadCalibrationData(void )
{
  // intrisicMat.at<double>(0, 0) = 337.210073178;
  // intrisicMat.at<double>(1, 0) = 0;
  // intrisicMat.at<double>(2, 0) = 0;

  // intrisicMat.at<double>(0, 1) = 0;
  // intrisicMat.at<double>(1, 1) = 432.9718978095;
  // intrisicMat.at<double>(2, 1) = 0;

  // intrisicMat.at<double>(0, 2) = 320;
  // intrisicMat.at<double>(1, 2) = 240;
  // intrisicMat.at<double>(2, 2) = 1;

  // rVec.at<double>(0) = 0;
  // rVec.at<double>(1) = 0;
  // rVec.at<double>(2) = 0;

  // tVec.at<double>(0) = 0;
  // tVec.at<double>(1) = 0;
  // tVec.at<double>(2) = 0.3;

  // distCoeffs.at<double>(0) = 0;
  // distCoeffs.at<double>(1) = 0;
  // distCoeffs.at<double>(2) = 0;
  // distCoeffs.at<double>(3) = 0;
  // distCoeffs.at<double>(4) = 0;

  //可参考https://blog.csdn.net/weixin_45377028/article/details/109194773
  // 激光雷达到相机的变换矩阵
  // Eigen::Isometry3d T= Eigen::Isometry3d::Identity();
  // T.pretranslate(Eigen::Vector3d(0,0,0.3));
  // Eigen::AngleAxisd r1(-M_PI/2, Eigen::Vector3d(0,1,0));
  // T.rotate(r1);
  // Eigen::AngleAxisd r2(M_PI/2, Eigen::Vector3d(0,0,1));
  // T.rotate(r2);
  // std::cout << T.matrix() << std::endl;

  RT.at<double>(0,0) = 0.0; RT.at<double>(0,1) = -1; RT.at<double>(0,2) = 0; RT.at<double>(0,3) =0;
  RT.at<double>(1,0) = 0.0; RT.at<double>(1,1) = 0.0; RT.at<double>(1,2) = -1; RT.at<double>(1,3) = 0.3;
  RT.at<double>(2,0) = 1.0; RT.at<double>(2,1) = 0.0; RT.at<double>(2,2) = 0.0; RT.at<double>(2,3) = 0;
  RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;

  //相机的内参矩阵
  P_rect_00.at<double>(0,0) =337.2084410968044; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 320.5; P_rect_00.at<double>(0,3) = 0.000000e+00;  
  P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 337.2084410968044; P_rect_00.at<double>(1,2) = 240.5; P_rect_00.at<double>(1,3) = 0.000000e+00;  
  P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;
 //337.210073178    432.9718978095
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_fusion");
  // cv::namedWindow("view");
  // cv::startWindowThread();
  loadCalibrationData();
  pcl_fusion pf;
  ros::NodeHandle n;
  image_transport::ImageTransport it(n); //用之前声明的节点句柄初始化it，其实这里的it和nh的功能基本一样，可以像之前一样使用it来发布和订阅相消息。
  image_transport::Subscriber sub = it.subscribe("/realsense/color/image_raw", 1, &imageCallback);
  
  ros::spin();
  // cv::destroyWindow("view");
}


