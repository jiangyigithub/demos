#include <lidar_msgs/ProjectedSpin.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <driving_common/velodyne_transformer.h>
#include <driving_common/velodyne_message_filter.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace velodyne {

class VelodynePointCloudNodelet : public nodelet::Nodelet
{
public:
    VelodynePointCloudNodelet() :
        velodyne_filter_(NULL),
        velodyne_sub_(NULL),
        tf_filter_(NULL)
    {
      ROS_WARN("Velodyne point cloud nodelet is deprecated. You should not use this code");
    }

    ~VelodynePointCloudNodelet()
    {
        unsubscribe();
    }

private:
    boost::recursive_mutex connection_mutex_;

    void onInit()
    {
        nh_ = getNodeHandle();

        ros::AdvertiseOptions ao;
        ao.init<pcl::PointCloud<pcl::PointXYZI> >("projected_points", 5, 
                                                  boost::bind(&VelodynePointCloudNodelet::onConnect, this),
                                                  boost::bind(&VelodynePointCloudNodelet::onConnect, this));

        // Lazy subscription
        // Need to lock mutex here to prevent race conditions on startup
        boost::lock_guard<boost::recursive_mutex> lock(connection_mutex_);
        pcl_pub_ = nh_.advertise(ao);
    }

    void onConnect()
    {
        boost::lock_guard<boost::recursive_mutex> lock(connection_mutex_);
      
        if (pcl_pub_.getNumSubscribers()) 
           subscribe();
        else
           unsubscribe();
    }

    void unsubscribe()
    {
      delete tf_filter_;
      delete velodyne_filter_;
      delete velodyne_sub_;

      tf_filter_       = NULL;
      velodyne_filter_ = NULL;
      velodyne_sub_    = NULL;
    }

    void subscribe()
    {
        if (velodyne_sub_ || velodyne_filter_ || tf_filter_)
           return;

        velodyne_sub_ = new message_filters::Subscriber<lidar_msgs::ProjectedSpin>();
        velodyne_sub_->subscribe(nh_, "scan", 100);

        velodyne_filter_ = new driving_common::VelodyneScanMessageFilter(*velodyne_sub_);
        velodyne_filter_->setCutAngle(M_PI);

        tf_filter_ = new tf::MessageFilter<lidar_msgs::ProjectedSpin>(tfl_, "odom", 1, nh_);
        tf_filter_->connectInput(*velodyne_filter_);
        tf_filter_->registerCallback(boost::bind(&VelodynePointCloudNodelet::velodyneCB, this, _1));
    }

    void velodyneCB(const lidar_msgs::ProjectedSpinConstPtr &spin)
    {
        lidar_msgs::ProjectedSpin spin_deskewed;
        try
        {
          tf::StampedTransform odom_to_veh_origin;
          tfl_.lookupTransform(spin->header.frame_id, "/odom", spin->header.stamp, odom_to_veh_origin);
          driving_common::deskewVelodyneScan("/odom", tfl_, odom_to_veh_origin, *spin, spin_deskewed);
        }
        catch (std::runtime_error &ex)
        {
          ROS_ERROR("Unable to deskew Velodyne scan!");
          return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        driving_common::velodyneScanToPointCloud(spin_deskewed, *cloud);

        pcl_pub_.publish(cloud);
    }

    ros::NodeHandle nh_;

    ros::Publisher pcl_pub_;

    tf::TransformListener tfl_;

    driving_common::VelodyneScanMessageFilter                  *velodyne_filter_;
    message_filters::Subscriber<lidar_msgs::ProjectedSpin> *velodyne_sub_;
    tf::MessageFilter<lidar_msgs::ProjectedSpin>           *tf_filter_;
};

PLUGINLIB_DECLARE_CLASS(velodyne, VelodynePointCloudNodelet, velodyne::VelodynePointCloudNodelet, nodelet::Nodelet);

} // namespace
