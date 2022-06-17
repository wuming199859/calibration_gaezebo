#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE


#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

/*
    *一个具有XYZ、intensity、ring的点云类型
    */
struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)


/*
    * 一个具有XYZ、RGB、intensity、ring的点云类型
    */
struct PointXYZRGBIR
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    uint16_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBIR,  
                            (float, x, x)
                            (float, y, y)
                            (float, z, z)
                            (float, rgb, rgb)
                            (float, intensity, intensity)
                            (uint16_t, label, label)
                            (uint16_t, ring, ring)
)
 
#endif  //PCL_NO_PRECOMPILE


