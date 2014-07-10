#ifndef TEDUSAR_BOX_DETECTION__BOX_DETECTION_H_
#define TEDUSAR_BOX_DETECTION__BOX_DETECTION_H_

// Prevent unaligned array exception:
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <moveit_msgs/PlanningScene.h>
#include <tedusar_box_detection_msgs/BoxDetectionAction.h>

namespace tedusar_box_detection
{

class BoxDetectionNode
{
public:
    BoxDetectionNode();

private:
    typedef pcl::PointXYZ PclPoint;
    typedef pcl::PointCloud<PclPoint> PclPointCloud;

    struct Parameters
    {
        std::string point_cloud_topic_;
        Eigen::Vector3f box_size_min_;
        Eigen::Vector3f box_size_max_;
        double detection_timeout_;
        int min_points_per_plane_;
        std::string target_frame_id_;
        std::string collision_objects_basename_;
        double plane_fitting_distance_threshold_;
        double plane_fitting_eps_angle_;
        int plane_fitting_max_iterations_;
        double downsampling_leaf_size_;
        double clusterization_tolerance_;
        bool have_plane_publisher_;
        bool have_collision_object_publisher_;
        bool have_visualization_marker_publisher_;
        double publishing_rate_;
        bool have_action_server_debug_output_;
        bool have_box_detection_debug_output_;
    };

    struct Box
    {
        unsigned int id_;
        std::string name_;
        geometry_msgs::PoseStamped pose_;
        geometry_msgs::Vector3 size_;
    };

    void loadParameters();
    template <typename T> static void getRequiredParameter(ros::NodeHandle & private_nh, const std::string & key, T & value);
    template <typename T> static void getOptionalParameter(ros::NodeHandle & private_nh, const std::string & key, T & value, T default_value);
    template <typename T> static bool getParameter(ros::NodeHandle & private_nh, const std::string & key, T & value);

    void boxDetectionActionGoalCallback();
    void boxDetectionActionPreemptCallback();
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg);
    void publisherTimerCallback(const ros::TimerEvent &);

    void fitPlane(const PclPointCloud::ConstPtr & cloud, pcl::PointIndices::Ptr & inliers);
    void extractPlane(const PclPointCloud::ConstPtr & cloud,
                      const pcl::PointIndices::ConstPtr & inliers,
                      PclPointCloud & cloud_plane,
                      PclPointCloud & cloud_rest);
    void downsampleCloud(const PclPointCloud::ConstPtr & cloud, PclPointCloud & cloud_filtered);
    void clusterizeCloud(const PclPointCloud::ConstPtr & cloud, std::vector<pcl::PointIndices> & cluster_indices);
    bool fitBox(const PclPointCloud::ConstPtr & cloud, Box & box);
    void publishClusterCloud(const PclPointCloud::ConstPtr & cloud);
    void publishCollisionObject(const Box & box);
    void publishVisualizationMarker(const Box & box);
    void publishActionFeedback(const Box & box);

    ros::NodeHandle nh_;
    Parameters parameters_;
    actionlib::SimpleActionServer<tedusar_box_detection_msgs::BoxDetectionAction> box_detection_as_;
    tf::TransformListener tf_listener_;
    ros::Subscriber point_cloud_subscriber_;
    ros::Publisher plane_cloud_publisher_;
    ros::Timer publisher_timer_;
    ros::Publisher planning_scene_publisher_;
    ros::Publisher visualization_marker_publisher_;
    ros::Time detection_timeout_time_;
    std::vector<Box> boxes_;
    sensor_msgs::PointCloud plane_cloud_;
    float plane_intensity_;
    unsigned int box_counter_;
};

}

#endif // TEDUSAR_BOX_DETECTION__BOX_DETECTION_H_
