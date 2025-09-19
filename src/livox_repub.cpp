#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"

#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver2/CustomMsg.h"

struct LivoxPoint {
    PCL_ADD_POINT4D;
    float intensity;
    uint8_t tag;
    uint8_t line;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (LivoxPoint,
(float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
(uint8_t, tag, tag)(uint8_t, line, line)
)


typedef LivoxPoint PointType;
//typedef pcl::PointCloud<PointType> PointCloudXYZI;

ros::Publisher pub_pcl_out0, pub_pcl_out1;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver2::CustomMsgConstPtr> livox_data;
float min_dist = -1.f;

void LivoxMsgCbk1(const livox_ros_driver2::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;

  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      if (min_dist > 0.0 && livox_msg->points[i].x < min_dist)
      {
        continue;
      }

      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      
      float s = livox_msg->points[i].offset_time / (float)time_end;

      pt.intensity = livox_msg->points[i].line +livox_msg->points[i].reflectivity; // The integer part is line number and the decimal part is timestamp
//      pt.curvature = s*0.1;
        pt.tag =    livox_msg->points[i].tag;
        pt.line =    livox_msg->points[i].line;

        pcl_in.push_back(pt);
    }
  }

  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);

  // pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  // pcl_ros_msg.header.frame_id = "/livox";
  pcl_ros_msg.header = livox_data[0]->header;

  pub_pcl_out1.publish(pcl_ros_msg);
  livox_data.clear();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh("~");

  ROS_INFO("start livox_repub");

  nh.getParam("min_dist", min_dist);
  ROS_INFO("Param min_dist : %f", min_dist);


  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver2::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCbk1);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar/pc2", 100);

  ros::spin();
}
