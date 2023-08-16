#include <iostream>
#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
// #include <sick_ldmrs_msgs/sick_ldmrs_point_type.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace pcl;

# define pcl_isfinite(x) std::isfinite(x)

struct TanwayPCLEXPoint
{
    PCL_ADD_POINT4D;
	float intensity;
  	int channel;
	float angle;
	int echo;
	int block;				/*For duetto*/
  	unsigned int t_sec;     /* The value represents seconds since 1900-01-01 00:00:00 (the UNIX epoch).*/ 
	unsigned int t_usec;    /* remaining microseconds */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(TanwayPCLEXPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (int, channel, channel)
                                  (float, angle, angle)	
                                  (int, echo, echo)
								  (int, block, block)
								  (unsigned int, t_sec, t_sec)
								  (unsigned int, t_usec, t_usec)
                                 )

typedef pcl::PointCloud<TanwayPCLEXPoint> PointCloudT;
typedef TanwayPCLEXPoint PointT;

std::string topic_pcl = "/tanwaylidar_undistort";

// typedef sick_ldmrs_msgs::SICK_LDMRS_Point PointT;
// typedef pcl::PointCloud<PointT> PointCloudT;

ros::Publisher pub;
ros::Publisher pub_bg;

double min_dist_ratio_;

PointCloudT::Ptr cloud_bg_;


void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  PointCloudT::Ptr cloudfg(new PointCloudT);
  pcl::fromROSMsg(*pc, *cloud);

//   if (!cloud->isOrganized())
//   {
//     ROS_ERROR_THROTTLE(15.0, "Input point cloud is not organized!");
//     return;
//   }

//   if (cloud_bg_ && (cloud->width != cloud_bg_->width || cloud->height != cloud_bg_->height))
//   {
//     ROS_INFO("Dimensions of input cloud different from stored background, resetting background.");
//     cloud_bg_.reset();
//   }
  
  if (!cloud_bg_)
  {
    cloud_bg_ = cloud;
    return;
  }

  PointT invalid;
  invalid.x = invalid.y = invalid.z = std::numeric_limits<float>::quiet_NaN();
//  PointCloudT::Ptr cloud_out = boost::make_shared<PointCloudT>(width, height, invalid);
//  cloud_out->is_dense = false;
  int pointNum = min({cloud->size(), cloud_bg_->size()});
  int invalidNum = 0;
  for (size_t i = 0; i < pointNum; i++)
  {
    const PointT &p_in = cloud->points[i];
    const PointT &p_bg = cloud_bg_->points[i];

    // if input point invalid -> output invalid
    if (!(pcl_isfinite(p_in.x) && pcl_isfinite(p_in.y) && pcl_isfinite(p_in.z)))
    {
        continue;
    }

    // if background invalid (and input valid) -> output valid
    if (!(pcl_isfinite(p_bg.x) && pcl_isfinite(p_bg.y) && pcl_isfinite(p_bg.z)))
    {
        (*cloud_bg_)[i] = p_in;     // store new background point
        continue;
    }

    // compare ratio between background and input point to threshold
    float dist_in = p_in.getVector3fMap().norm();
    float dist_bg = p_bg.getVector3fMap().norm();
    float diff = dist_bg - dist_in;

    if (diff < 0)
    {
      // point is further away than previous background
      (*cloud_bg_)[i] = p_in;
      cloudfg->push_back((*cloud)[i]);
      (*cloud)[i] = invalid;
      invalidNum++;
    }
    else if (diff / dist_bg < min_dist_ratio_)
    {
      // point is too close to background
      cloudfg->push_back((*cloud)[i]);
      (*cloud)[i] = invalid;
      invalidNum++;
    }
    std::cout<<invalidNum<<std::endl;
    // ... otherwise, point is foreground => don't invalidate
  }

//   sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header = pc->header;
  pub.publish(msg);

//   sensor_msgs::PointCloud2::Ptr msg_bg = boost::make_shared<sensor_msgs::PointCloud2>();
  sensor_msgs::PointCloud2 msg_bg;
  pcl::toROSMsg(*cloudfg, msg_bg);
  msg_bg.header = pc->header;
  pub_bg.publish(msg_bg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tanway_remove_background");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // min_dist_ratio: a point is filtered out if the distance between it and its
  // reference point is less than 5% of the distance between the sensor and the
  // reference point.
  min_dist_ratio_ = private_nh.param("min_dist_ratio", 0.1);

  ros::Subscriber sub = nh.subscribe(topic_pcl, 10, callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("foreground", 10);
  pub_bg = nh.advertise<sensor_msgs::PointCloud2>("background", 10);

  ros::spin();

  return 0;
}
