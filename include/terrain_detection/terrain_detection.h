#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h> // pub, sub images in ROS
#include <cv_bridge/cv_bridge.h> // useful functions for image encodings
#include <sensor_msgs/image_encodings.h> // for ROS-OpenCV conversion: toCvShare, toCvCopy
#include <opencv2/imgproc/imgproc.hpp> // image processing
#include <opencv2/highgui/highgui.hpp> // GUI modules

#include <iostream>

#include <dynamic_reconfigure/server.h>

#include <terrain_detection/TerrainDetectionConfig.h>

#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sstream>
#include <limits.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>

#include <geometry_msgs/TwistStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


#include <visualization_msgs/Marker.h>

#include <pluginlib/class_list_macros.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include "pcl_ros/transforms.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/pcl_search.h>
#include <pcl/filters/extract_indices.h>

#include "sensor_msgs/Imu.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>




typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef PointCloud::Ptr ptr_cloud;

using namespace cv;
using namespace cv_bridge; // CvImage, toCvShare


namespace terrain
{
  class TerrainDetectionNodelet : public nodelet::Nodelet
  {
    public:
      // ROS communication
      boost::shared_ptr<image_transport::ImageTransport> it_in_;
      image_transport::CameraSubscriber sub_low_depth_image_;
      ros::Subscriber sub_pcl_1;
      ros::Subscriber sub_pcl_2;

      ros::Publisher pub_output_;
      ros::Publisher pub_pcl_1;
      ros::Publisher pub_pcl_2;

      ros::NodeHandle nh;
      ros::NodeHandle private_nh;

      boost::mutex connect_mutex_;

      int queue_size_;
      std::string target_frame_id_;

      // Dynamic reconfigure
      boost::recursive_mutex config_mutex_;
      typedef terrain_detection::TerrainDetectionConfig Config;
      typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
      boost::shared_ptr<ReconfigureServer> reconfigure_server_;
      Config config_;

      void onInit();

      // Handles (un)subscribing when clients (un)subscribe;
      void connectCb();

      void configCb(Config &config, uint32_t level);

      void low_depth_image_cb(const sensor_msgs::ImageConstPtr& image_msg,
                  const sensor_msgs::CameraInfoConstPtr& info_msg);

      void pcl_1_cb(const PointCloud::ConstPtr& msg);
      void pcl_2_cb(const PointCloud::ConstPtr& msg);

      void depth_image_to_twist(const sensor_msgs::ImageConstPtr& image_msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg,
                                const ros::Publisher pub_pcl_,
                                int x_offset,
                                int y_offset,
                                int width,
                                int height);

      void point_cloud_to_twist(const PointCloud::ConstPtr& cloud_in,
                                float transform_pcl_yaw_,
                                float transform_pcl_pitch_,
                                float transform_pcl_roll_,
                                const ros::Publisher pub_pcl_,
                                long int seq);

    private:

      // Depth Camera Frames
      std::string low_robot_frame;
      std::string low_sensor_frame;

      // Depth Image Parameters
      int low_camera_pixel_x_offset;
      int low_camera_pixel_y_offset;
      int low_camera_pixel_width;
      int low_camera_pixel_height;

      double low_roll;
      double low_pitch;
      double low_yaw;

      // Point Cloud Sequence
      long int seq_1 = 0;
      long int seq_2 = 0;

      float transform_pcl_roll_1_;
      float transform_pcl_pitch_1_;
      float transform_pcl_yaw_1_;
      float transform_pcl_roll_2_;
      float transform_pcl_pitch_2_;
      float transform_pcl_yaw_2_;
      float normal_radius_;
      float normal_x_LT_threshold_;
      float normal_x_GT_threshold_;
      float normal_y_LT_threshold_;
      float normal_y_GT_threshold_;
      float normal_z_LT_threshold_;
      float normal_z_GT_threshold_;
      float ror_radius_;
      float ror_min_neighbors_;


  };

};
