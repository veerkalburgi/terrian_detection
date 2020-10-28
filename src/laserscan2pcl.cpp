#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;

        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;

        ros::Subscriber scan_sub_;

        std::string vehicle_name_;
};

My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("terrain_detection/scan_out_center", 100, &My_Filter::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("terrain_detection/laserscanPCL", 100, false);
        ros::NodeHandle nh("~");
        nh.param<std::string>("vehicle_name", vehicle_name_, "default_value");
        //tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud(vehicle_name_.c_str(), *scan, cloud, tfListener_);
    //cloud += cloud2_ + cloud3_;
    pcl::PointCloud<pcl::PointXYZ> cloud_pcl, cloud_pcl2, cloud_pcl3, cloud_pcl4;
    pcl::fromROSMsg (cloud, cloud_pcl);
    cloud_pcl2 = cloud_pcl;
    cloud_pcl3 = cloud_pcl;
    cloud_pcl4 = cloud_pcl;


    //ROS_INFO("%f", cloud_pcl.points.size());
    if (cloud_pcl.points.size() > 0){
        for(std::size_t i=0; i<cloud_pcl.points.size(); i++){
          cloud_pcl2.points[i].z = cloud_pcl.points[i].z+.2;
          cloud_pcl3.points[i].z = cloud_pcl.points[i].z+.4;
          cloud_pcl4.points[i].z = cloud_pcl.points[i].z+.6;
        }
        cloud_pcl2 += cloud_pcl;
        cloud_pcl2 += cloud_pcl3;
        cloud_pcl2 += cloud_pcl4;
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud_pcl2, output);
        point_cloud_publisher_.publish(output);
    }

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter");

    My_Filter filter;

    ros::spin();

    return 0;
}
