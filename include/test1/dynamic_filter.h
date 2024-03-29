#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

struct TanwayPCLEXPoint
{
    PCL_ADD_POINT4D;
	float intensity;
  	int channel;
	float angle;
	int echo;
	int block;				/* For duetto */
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

typedef pcl::PointCloud<TanwayPCLEXPoint> PointCloudEX;
typedef sensor_msgs::PointCloud2::ConstPtr& SPcPtr;

typedef struct
{
    // basic
    int pnum;
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    float zmin;
    float zmax;
    float zmean;

    // pca
    float d0[3];
    float d1[3];
    float center[3];
    float obb[8];

    // class
    int cls;

} SClusterFeature;

void FilterGnd(const sensor_msgs::PointCloud2::ConstPtr& Cloud);
SPcPtr FilterGnd_2(const sensor_msgs::PointCloud2::ConstPtr& Cloud);
// class DynamicFilter{
//     public:
//         void FilterGnd(PointCloudEX::Ptr& GndCloud,  PointCloudEX::Ptr& noGndCloud, sensor_msgs::PointCloud2::Ptr& Cloud, int inNum);
//         void SegBG(int *pLabel, PointCloudEX::Ptr inCloud, pcl::KdTreeFLANN<TanwayPCLEXPoint> &kdtree, float fSearchRadius);
//     // private:
// };