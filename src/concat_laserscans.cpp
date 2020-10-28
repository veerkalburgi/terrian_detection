#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Concat_Laserscan {
    public:
        Concat_Laserscan();
        void leftScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void rightScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void concatScans(const sensor_msgs::LaserScan left_scan, const sensor_msgs::LaserScan right_scan);
        void publishScanOut();

    private:
        ros::NodeHandle node_;
        ros::Publisher laserscan_publisher_;

        ros::Subscriber left_scan_sub_;
        ros::Subscriber right_scan_sub_;

        sensor_msgs::LaserScan last_left_scan_;
        sensor_msgs::LaserScan last_right_scan_;
        sensor_msgs::LaserScan scan_out_;
        sensor_msgs::LaserScan scan_out_temp_;

        ros::Time last_left_scan_time_;
        ros::Time last_right_scan_time_;

        //std::string vehicle_name_;
        int publish_rate_;
        double right_scan_limit_;
        double left_scan_limit_;

        int num_scan_points_;
        double scan_timeout_s_;

        bool have_left_scan_;
        bool have_right_scan_;
};

Concat_Laserscan::Concat_Laserscan(){
        left_scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("terrain_detection/scan_out_left", 100, &Concat_Laserscan::leftScanCallback, this);
        right_scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("terrain_detection/scan_out_right", 100, &Concat_Laserscan::rightScanCallback, this);
        laserscan_publisher_ = node_.advertise<sensor_msgs::LaserScan> ("terrain_detection/concat_scan_out", 100, false);
        ros::NodeHandle nh("~");
        nh.param("publish_rate", publish_rate_, 0);
        nh.param("num_scan_points", num_scan_points_, 90);
        nh.param("right_scan_limit", right_scan_limit_, 0.785);
        nh.param("left_scan_limit", left_scan_limit_, -0.785);
        nh.param("scan_timout_s", scan_timeout_s_, 0.25);
        auto tf_prefix = nh.param("vehicle_name", std::string{});
        //tf_prefix = "H01";
        auto scan_frame = tf_prefix + "/base_link";

        //ROS_INFO("%d", num_scan_points_);

        scan_out_.angle_min = left_scan_limit_;
        scan_out_.angle_max = right_scan_limit_;
        scan_out_.header.frame_id = scan_frame;
        scan_out_.angle_increment =  0.00873;
        scan_out_.range_max = 10.0;

        last_left_scan_time_ = ros::Time::now();
        last_right_scan_time_ = ros::Time::now();

        have_left_scan_ = false;
        have_right_scan_ = false;
}

void Concat_Laserscan::leftScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    //ROS_INFO("left_scan_cb");
    if(!have_left_scan_){
        have_left_scan_ = true;
    }
    last_left_scan_ = *scan;
    last_left_scan_time_ = ros::Time::now();
}

void Concat_Laserscan::rightScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    //ROS_INFO("right_scan_cb");
    if(!have_right_scan_){
        have_right_scan_ = true;
    }
    last_right_scan_ = *scan;
    last_right_scan_time_ = ros::Time::now();
}

void Concat_Laserscan::concatScans(const sensor_msgs::LaserScan left_scan, const sensor_msgs::LaserScan right_scan){
    scan_out_.ranges.clear();
    scan_out_temp_.ranges.clear();
    //ROS_INFO("test1");

    if(((ros::Time::now() - last_right_scan_time_).toSec() > scan_timeout_s_) || !have_right_scan_){
        // Haven't recieved a rightt scan in a while
        for(int i = 0; i < num_scan_points_; i++){
            scan_out_temp_.ranges.push_back(10.0);
        }
    }  else {
        for(int i = 0; i < num_scan_points_; i++){
            scan_out_temp_.ranges.push_back(right_scan.ranges[i]);
        }
    }

    if(((ros::Time::now() - last_left_scan_time_).toSec() > scan_timeout_s_) || !have_left_scan_){
        // Haven't recieved a left scan in a while
        for(int i = 0; i < num_scan_points_; i++){
            scan_out_temp_.ranges.push_back(10.0);
        }
    } else {
        for(int i = 0; i < num_scan_points_; i++){
            //ROS_INFO("%f", left_scan.ranges[i]);
            scan_out_temp_.ranges.push_back(left_scan.ranges[i]);
        }
    }
    for(int i = 0; i < 2*num_scan_points_; i++){
        scan_out_.ranges.push_back(scan_out_temp_.ranges[i]);
    }
}

void Concat_Laserscan::publishScanOut(){
    scan_out_.header.stamp = ros::Time::now();
    concatScans(last_left_scan_, last_right_scan_);
    laserscan_publisher_.publish(scan_out_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "concat_terrain_laserscan");
    Concat_Laserscan concat_laserscan;

    ros::Rate loop_rate(10);

    while(ros::ok()){
      ros::spinOnce();
      concat_laserscan.publishScanOut();
      loop_rate.sleep();
    }

    return 0;
}
