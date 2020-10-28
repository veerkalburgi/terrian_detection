#include <terrain_detection/terrain_detection.h>

namespace terrain
{
    void TerrainDetectionNodelet::connectCb()
    {
    };

    void TerrainDetectionNodelet::onInit()
    {
        nh         = getMTNodeHandle();
        private_nh = getMTPrivateNodeHandle();

        it_in_ .reset(new image_transport::ImageTransport(nh));

        // Read parameters
        private_nh.param("queue_size", queue_size_, 5);
        private_nh.param("target_frame_id", target_frame_id_, std::string());

        // Set up dynamic reconfigure
        reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
        ReconfigureServer::CallbackType f = boost::bind(&TerrainDetectionNodelet::configCb, this, _1, _2);
        reconfigure_server_->setCallback(f);

	    // Monitor whether anyone is subscribed to the h_scans
        image_transport::SubscriberStatusCallback connect_cb = boost::bind(&TerrainDetectionNodelet::connectCb, this);
        ros::SubscriberStatusCallback connect_cb_info = boost::bind(&TerrainDetectionNodelet::connectCb, this);

        // Make sure we don't enter connectCb() between advertising and assigning to pub_h_scans_
        boost::lock_guard<boost::mutex> lock(connect_mutex_);
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());

        // Subscriber RealSense depth image
        sub_low_depth_image_ = it_in_->subscribeCamera("in_image_raw", queue_size_, &TerrainDetectionNodelet::low_depth_image_cb, this);

        sub_pcl_1 = nh.subscribe("pcl_in_1", 1, &TerrainDetectionNodelet::pcl_1_cb, this);
        sub_pcl_2 = nh.subscribe("pcl_in_2", 1, &TerrainDetectionNodelet::pcl_2_cb, this);

        // Publish Point Cloud
        pub_pcl_1    = private_nh.advertise<PointCloud>("pcl_out_1", 10);
        pub_pcl_2    = private_nh.advertise<PointCloud>("pcl_out_2", 10);
    };

    void TerrainDetectionNodelet::configCb(Config &config, uint32_t level)
    {
        config_ = config;

        // Depth Camera Parameters
        low_robot_frame            = config.low_robot_frame;
        low_sensor_frame           = config.low_sensor_frame;

        // Depth Image Parameters
        low_camera_pixel_x_offset  = config.low_camera_pixel_x_offset;
        low_camera_pixel_y_offset  = config.low_camera_pixel_y_offset;
        low_camera_pixel_width     = config.low_camera_pixel_width;
        low_camera_pixel_height    = config.low_camera_pixel_height;

        transform_pcl_roll_1_       = config.transform_pcl_roll_1;
        transform_pcl_pitch_1_      = config.transform_pcl_pitch_1;
        transform_pcl_yaw_1_        = config.transform_pcl_yaw_1;
        transform_pcl_roll_2_       = config.transform_pcl_roll_2;
        transform_pcl_pitch_2_      = config.transform_pcl_pitch_2;
        transform_pcl_yaw_2_        = config.transform_pcl_yaw_2;
        normal_radius_              = config.normal_radius;
        normal_x_LT_threshold_      = config.normal_x_LT_threshold;
        normal_x_GT_threshold_      = config.normal_x_GT_threshold;
        normal_y_LT_threshold_      = config.normal_y_LT_threshold;
        normal_y_GT_threshold_      = config.normal_y_GT_threshold;
        normal_z_LT_threshold_      = config.normal_z_LT_threshold;
        normal_z_GT_threshold_      = config.normal_z_GT_threshold;
        ror_radius_                 = config.ror_radius;
        ror_min_neighbors_          = config.ror_min_neighbors;

    };

    void TerrainDetectionNodelet::low_depth_image_cb(const sensor_msgs::ImageConstPtr& image_msg,
                                                     const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
    };

    void TerrainDetectionNodelet::pcl_1_cb(const PointCloud::ConstPtr& cloud_in)
    {
                
        point_cloud_to_twist(cloud_in,
                             transform_pcl_yaw_1_,
                             transform_pcl_pitch_1_,
                             transform_pcl_roll_1_,
                             pub_pcl_1,
                             seq_1
                            );
        
    };

    void TerrainDetectionNodelet::pcl_2_cb(const PointCloud::ConstPtr& cloud_in)
    {
                
        point_cloud_to_twist(cloud_in,
                             transform_pcl_yaw_2_,
                             transform_pcl_pitch_2_,
                             transform_pcl_roll_2_,
                             pub_pcl_2,
                             seq_2
                            );
        
    };

    void TerrainDetectionNodelet::depth_image_to_twist(const sensor_msgs::ImageConstPtr& image_msg,
                                                       const sensor_msgs::CameraInfoConstPtr& info_msg,
                                                       const ros::Publisher pub_pcl_,
                                                       int x_offset,
                                                       int y_offset,
                                                       int width,
                                                       int height)
    {
    };

    void TerrainDetectionNodelet::point_cloud_to_twist(const PointCloud::ConstPtr& cloud_in,
                                                       float transform_pcl_yaw_,
                                                       float transform_pcl_pitch_,
                                                       float transform_pcl_roll_,
                                                       const ros::Publisher pub_pcl_,
                                                       long int seq)
    {
        bool empty_publish = 1;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZINormal>);

        if(cloud_in->size() == 0)
        {
            // do nothing
        }
        else
        {
            //////////////////////////////////////
            // FILTER OUT NANS FROM POINT CLOUD //
            //////////////////////////////////////

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            boost::shared_ptr<std::vector<int>> indices_nan(new std::vector<int>);
            pcl::removeNaNFromPointCloud(*cloud_in, *indices_nan);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_in);
            extract.setIndices(indices_nan);
            extract.setNegative(false);
            extract.filter(*cloud_in_filtered);

            if(cloud_in_filtered->size() <= 1)
            {
                // do nothing
            }
            else
            {
                /////////////////////////////////
                // TRANSFORM INPUT POINT CLOUD //
                /////////////////////////////////

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_transformed (new pcl::PointCloud<pcl::PointXYZ>);

                // Note roll and pitch are intentionally backwards due to the image frame to boldy frame transform_pcl. The IMU transform_pcl converts from body to world frame.
                tf::Transform transform_pcl;
                transform_pcl.setRotation(tf::createQuaternionFromRPY(transform_pcl_roll_, transform_pcl_pitch_, transform_pcl_yaw_));
                pcl_ros::transformPointCloud(*cloud_in_filtered, *cloud_in_transformed, transform_pcl);

                ///////////////////////
                // CALCULATE NORMALS //
                ///////////////////////

                pcl::search::Search<pcl::PointXYZ>::Ptr tree_xyz;

                if (cloud_in_transformed->isOrganized ())
                {
                    tree_xyz.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
                }
                else
                {
                    // Use KDTree for non-organized data
                    tree_xyz.reset(new pcl::search::KdTree<pcl::PointXYZ> (false));
                }

                // Set input pointcloud for search tree
                tree_xyz->setInputCloud(cloud_in_transformed);

                pcl::NormalEstimation<pcl::PointXYZ, pcl::PointXYZINormal> ne; // NormalEstimationOMP uses more CPU on NUC (do not use OMP!)
                ne.setInputCloud(cloud_in_transformed);
                ne.setSearchMethod(tree_xyz);

                // Set viewpoint, very important so normals are all pointed in the same direction
                ne.setViewPoint(std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ne (new pcl::PointCloud<pcl::PointXYZINormal>);

                ne.setRadiusSearch(normal_radius_);
                ne.compute(*cloud_ne);

                // Assign original xyz data to normal estimate cloud (this is necessary because by default the xyz fields are empty)
                for (int i = 0; i < cloud_in_transformed->points.size(); i++)
                {
                    cloud_ne->points[i].x = cloud_in_transformed->points[i].x;
                    cloud_ne->points[i].y = cloud_in_transformed->points[i].y;
                    cloud_ne->points[i].z = cloud_in_transformed->points[i].z;
                }

                // Create conditional object
                pcl::ConditionOr<pcl::PointXYZINormal>::Ptr range_cond ( new pcl::ConditionOr<pcl::PointXYZINormal> () );

                // Add conditional statements
                range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_x",  pcl::ComparisonOps::LT, normal_x_LT_threshold_)) );
                range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_x",  pcl::ComparisonOps::GT, normal_x_GT_threshold_)) );
                range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_y",  pcl::ComparisonOps::LT, normal_y_LT_threshold_)) );
                range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_y",  pcl::ComparisonOps::GT, normal_y_GT_threshold_)) );
                range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_z",  pcl::ComparisonOps::LT, normal_z_LT_threshold_)) );
                range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_z",  pcl::ComparisonOps::GT, normal_z_GT_threshold_)) );

                // Build the filter
                pcl::ConditionalRemoval<pcl::PointXYZINormal> condrem;
                condrem.setCondition (range_cond);
                condrem.setInputCloud (cloud_ne);

                // Apply filter
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_condrem (new pcl::PointCloud<pcl::PointXYZINormal>);
                condrem.filter (*cloud_condrem);

                //////////////////
                // REDUCE NOISE //
                //////////////////

                if(cloud_condrem->size() == 0)
                {
                    // do nothing
                }
                else
                {
                    // Build the filter
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ror (new pcl::PointCloud<pcl::PointXYZINormal>);
                    pcl::RadiusOutlierRemoval<pcl::PointXYZINormal> ror;
                    ror.setInputCloud(cloud_condrem);
                    ror.setRadiusSearch(ror_radius_);
                    ror.setMinNeighborsInRadius (ror_min_neighbors_);
                    ror.filter (*cloud_ror);

                    // Note roll and pitch are intentionally backwards due to the image frame to boldy frame transform_pcl. The IMU transform_pcl converts from body to world frame.
                    tf::Transform transform_pcl_inv;
                    transform_pcl_inv.setRotation(tf::createQuaternionFromRPY(-transform_pcl_roll_, -transform_pcl_pitch_, -transform_pcl_yaw_));
                    pcl_ros::transformPointCloud(*cloud_ror, *cloud_out, transform_pcl.inverse());
                }
            }
        }

        //////////////////////////////////
        // PUBLISH FILTERED POINT CLOUD //
        //////////////////////////////////

        seq++;
        cloud_out->header.seq = seq;
        cloud_out->header.frame_id = cloud_in->header.frame_id;
        pcl_conversions::toPCL(ros::Time::now(), cloud_out->header.stamp);
        pub_pcl_.publish (cloud_out);

        // Clear memory
        cloud_out->clear();
        
    };
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( terrain::TerrainDetectionNodelet, nodelet::Nodelet)
