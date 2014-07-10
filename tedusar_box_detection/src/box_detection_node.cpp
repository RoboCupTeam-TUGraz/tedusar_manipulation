#include <tedusar_box_detection/box_detection_node.h>
#include <iostream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/MarkerArray.h>


static const std::string VISUALIZATION_MARKER_NAMESPACE = "tedusar_box_detection";


static bool operator<=(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2)
{
    return v1(0) <= v2(0) && v1(1) <= v2(1) && v1(2) <= v2(2);
}

namespace tedusar_box_detection
{

BoxDetectionNode::BoxDetectionNode()
    : box_detection_as_(nh_, "detect_boxes", false), plane_intensity_(0), box_counter_(0)
{
    loadParameters();

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    if (parameters_.have_collision_object_publisher_)
        planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    if (parameters_.have_visualization_marker_publisher_)
        visualization_marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    if (parameters_.have_plane_publisher_)
    {
        plane_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud>("plane_cloud", 1);
        publisher_timer_ = nh_.createTimer(ros::Rate(parameters_.publishing_rate_), &BoxDetectionNode::publisherTimerCallback, this);
    }

    box_detection_as_.registerGoalCallback(boost::bind(&BoxDetectionNode::boxDetectionActionGoalCallback, this));
    box_detection_as_.registerPreemptCallback(boost::bind(&BoxDetectionNode::boxDetectionActionPreemptCallback, this));
    box_detection_as_.start();

    if (parameters_.have_action_server_debug_output_)
        ROS_INFO("Action server started, waiting for goal");
}

void BoxDetectionNode::loadParameters()
{
    ros::NodeHandle private_nh("~");
    getOptionalParameter(private_nh, "point_cloud_topic", parameters_.point_cloud_topic_, std::string("/camera/depth/points"));
    getOptionalParameter(private_nh, "box_size_min", parameters_.box_size_min_, Eigen::Vector3f(0.26, 0.07, 0.0));
    getOptionalParameter(private_nh, "box_size_max", parameters_.box_size_max_, Eigen::Vector3f(0.45, 0.12, 0.05));
    getOptionalParameter(private_nh, "detection_timeout", parameters_.detection_timeout_, 30.0);
    getOptionalParameter(private_nh, "min_points_per_plane", parameters_.min_points_per_plane_, 100);
    getOptionalParameter(private_nh, "target_frame_id", parameters_.target_frame_id_, std::string("base_link"));
    getOptionalParameter(private_nh, "collision_objects_basename", parameters_.collision_objects_basename_, std::string("box"));
    getOptionalParameter(private_nh, "plane_fitting_distance_threshold", parameters_.plane_fitting_distance_threshold_, 0.02);
    getOptionalParameter(private_nh, "plane_fitting_eps_angle", parameters_.plane_fitting_eps_angle_, 0.05);
    getOptionalParameter(private_nh, "plane_fitting_max_iterations", parameters_.plane_fitting_max_iterations_, 50);
    getOptionalParameter(private_nh, "downsampling_leaf_size", parameters_.downsampling_leaf_size_, 0.005);
    getOptionalParameter(private_nh, "clusterization_tolerance", parameters_.clusterization_tolerance_, 0.02);
    getOptionalParameter(private_nh, "have_plane_publisher", parameters_.have_plane_publisher_, true);
    getOptionalParameter(private_nh, "have_collision_object_publisher", parameters_.have_collision_object_publisher_, true);
    getOptionalParameter(private_nh, "have_visualization_marker_publisher", parameters_.have_visualization_marker_publisher_, true);
    getOptionalParameter(private_nh, "publishing_rate", parameters_.publishing_rate_, 2.0);
    getOptionalParameter(private_nh, "have_action_server_debug_output", parameters_.have_action_server_debug_output_, true);
    getOptionalParameter(private_nh, "have_box_detection_debug_output", parameters_.have_box_detection_debug_output_, true);
}

template <typename T>
void BoxDetectionNode::getRequiredParameter(ros::NodeHandle & private_nh, const std::string & key, T & value)
{
    if (!getParameter(private_nh, key, value))
    {
        ROS_FATAL_STREAM("Missing required node parameter " << key);
        throw std::runtime_error("Missing required node parameter " + key);
    }
}

template <typename T>
void BoxDetectionNode::getOptionalParameter(ros::NodeHandle & private_nh, const std::string & key, T & value, T default_value)
{
    if (!getParameter(private_nh, key, value))
    {
        value = default_value;
        ROS_WARN_STREAM("Missing optional node parameter " << key << ", using default value " << default_value);
    }
}

template <typename T> bool BoxDetectionNode::getParameter(ros::NodeHandle & private_nh, const std::string & key, T & value)
{
    return private_nh.getParam(key, value);
}

template <> bool BoxDetectionNode::getParameter(ros::NodeHandle & private_nh, const std::string & key, Eigen::Vector3f & value)
{
    std::string value_as_string;
    if (private_nh.getParam(key, value_as_string))
    {
        std::vector<std::string> parts;
        boost::split(parts, value_as_string, boost::is_any_of(" \t"));
        if (parts.size() == 3)
        {
            for (size_t i = 0; i < parts.size(); ++i)
            {
                std::istringstream ss(parts.at(i));
                ss >> value(i);
                if (ss.fail())
                    return false;
            }
            return true;
        }
    }
    return false;
}

void BoxDetectionNode::boxDetectionActionGoalCallback()
{
    if (parameters_.have_action_server_debug_output_)
        ROS_INFO("BoxDetection action started");

    box_detection_as_.acceptNewGoal();

    point_cloud_subscriber_ = nh_.subscribe(parameters_.point_cloud_topic_, 1, &BoxDetectionNode::pointCloudCallback, this);

    detection_timeout_time_ = ros::Time::now() + ros::Duration(parameters_.detection_timeout_);

    plane_cloud_ = sensor_msgs::PointCloud();
    plane_cloud_.header.frame_id = parameters_.target_frame_id_;
    plane_cloud_.channels.resize(1);
    plane_cloud_.channels.at(0).name = "intensity";
    plane_intensity_ = 0;

    // Delete all visualization markers:
    if (visualization_marker_publisher_ && !boxes_.empty())
    {
        visualization_msgs::MarkerArrayPtr markers = boost::make_shared<visualization_msgs::MarkerArray>();
        markers->markers.resize(boxes_.size());
        for (size_t i = 0; i < boxes_.size(); ++i)
        {
            visualization_msgs::Marker & marker = markers->markers[i];
            marker.ns = VISUALIZATION_MARKER_NAMESPACE;
            marker.id = boxes_[i].id_;
            marker.header.frame_id = boxes_[i].pose_.header.frame_id;
            marker.action = visualization_msgs::Marker::DELETE;
        }
        visualization_marker_publisher_.publish(markers);
    }

    boxes_.clear();
}

void BoxDetectionNode::boxDetectionActionPreemptCallback()
{
    box_detection_as_.setPreempted();
    if (parameters_.have_action_server_debug_output_)
        ROS_INFO("BoxDetection action preempted");
}

void BoxDetectionNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    point_cloud_subscriber_.shutdown();

    try
    {
      tf_listener_.waitForTransform(parameters_.target_frame_id_, msg->header.frame_id, ros::Time::now(), ros::Duration(10.0));
    }
    catch (tf::TransformException & ex)
    {
      ROS_ERROR_STREAM("TransformException: " << ex.what());
      box_detection_as_.setAborted(tedusar_box_detection_msgs::BoxDetectionResult(), "tf transformation failed");
      return;
    }

    PclPointCloud::Ptr cloud = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr transformed_cloud = boost::make_shared<PclPointCloud>();

    pcl::fromROSMsg(*msg, *cloud);
    pcl_ros::transformPointCloud(parameters_.target_frame_id_, *cloud, *transformed_cloud, tf_listener_);

    do
    {
        PclPointCloud::Ptr plane_cloud = boost::make_shared<PclPointCloud>();
        PclPointCloud::Ptr remaining_cloud = boost::make_shared<PclPointCloud>();
        PclPointCloud::Ptr downsampled_plane_cloud = boost::make_shared<PclPointCloud>();

        pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
        fitPlane(transformed_cloud, inliers);

        if (inliers->indices.size() < parameters_.min_points_per_plane_)
            break; // Plane isn't even big enough for one box => stop

        if (parameters_.have_box_detection_debug_output_)
            ROS_INFO("Found plane with %d points", inliers->indices.size());

        extractPlane(transformed_cloud, inliers, *plane_cloud, *remaining_cloud);
        downsampleCloud(plane_cloud, *downsampled_plane_cloud);

        if (parameters_.have_box_detection_debug_output_)
            ROS_INFO("Downsampled it to %d points", downsampled_plane_cloud->size());

        std::vector<pcl::PointIndices> cluster_indices;
        clusterizeCloud(downsampled_plane_cloud, cluster_indices);

        if (parameters_.have_box_detection_debug_output_)
            ROS_INFO("Clustered into %d clusters", cluster_indices.size());

        pcl::PointIndices::Ptr point_indices = boost::make_shared<pcl::PointIndices>();
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::ExtractIndices<PclPoint> extractor;
            extractor.setInputCloud(downsampled_plane_cloud);
            *point_indices = *it;
            extractor.setIndices(point_indices); // Must be done after assigning new indices to point_indices!

            PclPointCloud::Ptr cloud_cluster = boost::make_shared<PclPointCloud>();
            extractor.filter(*cloud_cluster);
            if (parameters_.have_box_detection_debug_output_)
                ROS_INFO("Checking cluster with %d points", cloud_cluster->size());

            publishClusterCloud(cloud_cluster);

            // Check if we found a box:
            Box box;
            if (fitBox(cloud_cluster, box))
            {
                boxes_.push_back(box);

                if (parameters_.have_collision_object_publisher_)
                    publishCollisionObject(box);
                if (parameters_.have_visualization_marker_publisher_)
                    publishVisualizationMarker(box);
                publishActionFeedback(box);

                if (parameters_.have_box_detection_debug_output_)
                    ROS_INFO_STREAM("FOUND BOX! Name: " << box.name_ << "; pose: " << box.pose_.pose << "; size: " << box.size_);
            }
        }

        transformed_cloud = remaining_cloud;
    }
    while (box_detection_as_.isActive() && ros::Time::now() < detection_timeout_time_);

    if (box_detection_as_.isActive())
    {
        tedusar_box_detection_msgs::BoxDetectionResult result;
        for (std::vector<Box>::iterator box_it = boxes_.begin(); box_it != boxes_.end(); ++box_it)
        {
            result.box_names.push_back(box_it->name_);
            result.box_poses.push_back(box_it->pose_);
        }
        box_detection_as_.setSucceeded(result);
    }
    if (parameters_.have_action_server_debug_output_)
        ROS_INFO("BoxDetection action finished; found %d boxes", boxes_.size());
}


void BoxDetectionNode::publisherTimerCallback(const ros::TimerEvent &)
{
    plane_cloud_publisher_.publish(plane_cloud_);
}


void BoxDetectionNode::fitPlane(const PclPointCloud::ConstPtr & cloud, pcl::PointIndices::Ptr & inliers)
{
    pcl::SACSegmentation<PclPoint> segmentation;

    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ()); // We assume we look at the boxes from the top.
    segmentation.setEpsAngle(parameters_.plane_fitting_eps_angle_);
    segmentation.setMaxIterations(parameters_.plane_fitting_max_iterations_);
    segmentation.setDistanceThreshold(parameters_.plane_fitting_distance_threshold_);
    segmentation.setInputCloud(cloud);

    //seg.setOptimizeCoefficients(true);

    pcl::ModelCoefficients coefficients;
    segmentation.segment(*inliers, coefficients);
}

void BoxDetectionNode::extractPlane(const PclPointCloud::ConstPtr & cloud,
                                    const pcl::PointIndices::ConstPtr & inliers,
                                    PclPointCloud & cloud_plane,
                                    PclPointCloud & cloud_rest)
{
    pcl::ExtractIndices<PclPoint> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(cloud_plane);

    extract.setNegative(true);
    extract.filter(cloud_rest);
}

void BoxDetectionNode::downsampleCloud(const PclPointCloud::ConstPtr & cloud, PclPointCloud & cloud_filtered)
{
    pcl::VoxelGrid<PclPoint> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(parameters_.downsampling_leaf_size_, parameters_.downsampling_leaf_size_, parameters_.downsampling_leaf_size_);
    filter.filter(cloud_filtered);
}

void BoxDetectionNode::clusterizeCloud(const PclPointCloud::ConstPtr & cloud, std::vector<pcl::PointIndices> & cluster_indices)
{
    pcl::search::KdTree<PclPoint>::Ptr tree = boost::make_shared<pcl::search::KdTree<PclPoint> >();
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PclPoint> ece;
    ece.setInputCloud(cloud);
    ece.setSearchMethod(tree);
    ece.setClusterTolerance(parameters_.clusterization_tolerance_);
    ece.setMinClusterSize(parameters_.min_points_per_plane_);

    ece.extract(cluster_indices);
}

bool BoxDetectionNode::fitBox(const PclPointCloud::ConstPtr & cloud, Box & box)
{
    // Compute eigen vectors of cloud, which is basically the orientation of the cloud (first eigen vector points in the direction of the largest extent):
    pcl::PCA<PclPoint> pca;
    pca.setInputCloud(cloud);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

    // Transform cloud into eigen vector space:
    PclPointCloud projected_cloud;
    pca.project(*cloud, projected_cloud);

    // Get size of projected plane:
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(projected_cloud, min_pt, max_pt);
    Eigen::Vector3f size = max_pt.topRows(3) - min_pt.topRows(3);

    // Fail if cloud size is outside sensible limits:
    if (!(parameters_.box_size_min_ <= size && size <= parameters_.box_size_max_))
        return false;

    size.z() = size.y(); // We don't get the box's height from the plane, so we simply assume a square base

    // Compute center = center of plane minus height/2:
    float cos_between_z_axes = eigen_vectors.col(2).dot(Eigen::Vector3f::UnitZ()); // We have to check whether z axis points up or down
    float z_orientation = (cos_between_z_axes > 0) - (cos_between_z_axes < 0); // Signum of cosine (don't ask...)
    Eigen::Vector3f center = pca.getMean().topRows(3) - z_orientation * eigen_vectors.transpose() * Eigen::Vector3f(0.0f, 0.0f, size.z() / 2.0f);

    // Compute orientation (this is a bit simplified, I didn't want to think about quaternion computations...):
    Eigen::Quaternion<float> orientation;
    orientation.setFromTwoVectors(Eigen::Vector3f::UnitX(), eigen_vectors.col(0));

    // Generate id and name and put data into Box structure:
    box.id_ = box_counter_++;

    std::ostringstream ss;
    ss << parameters_.collision_objects_basename_ << box.id_;
    box.name_ = ss.str();

    box.pose_.header.frame_id = parameters_.target_frame_id_;
    box.pose_.header.stamp = ros::Time::now();
    box.pose_.pose.position.x = center(0);
    box.pose_.pose.position.y = center(1);
    box.pose_.pose.position.z = center(2);
    box.pose_.pose.orientation.x = orientation.x();
    box.pose_.pose.orientation.y = orientation.y();
    box.pose_.pose.orientation.z = orientation.z();
    box.pose_.pose.orientation.w = orientation.w();

    box.size_.x = size.x();
    box.size_.y = size.y();
    box.size_.z = size.z();

    return true;
}

void BoxDetectionNode::publishClusterCloud(const PclPointCloud::ConstPtr & cloud)
{
    // Add cluster to plane cloud:
    plane_intensity_ += 1.0f;
    for (PclPointCloud::const_iterator p_it = cloud->begin(); p_it != cloud->end(); ++p_it)
    {
        geometry_msgs::Point32 point;
        point.x = p_it->x;
        point.y = p_it->y;
        point.z = p_it->z;
        plane_cloud_.points.push_back(point);
        plane_cloud_.channels.at(0).values.push_back(plane_intensity_);
    }
}

void BoxDetectionNode::publishCollisionObject(const Box & box)
{
    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = box.size_.x;
    primitive.dimensions[1] = box.size_.y;
    primitive.dimensions[2] = box.size_.z;

    moveit_msgs::CollisionObject object;
    object.header = box.pose_.header;
    object.id = box.name_;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(box.pose_.pose);
    object.operation = moveit_msgs::CollisionObject::ADD;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.world.collision_objects.push_back(object);
    planning_scene_publisher_.publish(planning_scene);
}

void BoxDetectionNode::publishVisualizationMarker(const Box & box)
{
    visualization_msgs::MarkerArrayPtr markers = boost::make_shared<visualization_msgs::MarkerArray>();
    markers->markers.resize(1);
    visualization_msgs::Marker & marker = markers->markers.at(0);
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.ns = VISUALIZATION_MARKER_NAMESPACE;
    marker.id = box.id_;
    marker.header.frame_id = box.pose_.header.frame_id;
    marker.pose = box.pose_.pose;
    marker.scale = box.size_;
    marker.color.r = 0.8f;
    marker.color.g = 0.8f;
    marker.color.b = 0.2f;
    marker.color.a = 0.5f;
    visualization_marker_publisher_.publish(markers);
}

void BoxDetectionNode::publishActionFeedback(const Box & box){
    tedusar_box_detection_msgs::BoxDetectionFeedback feedback;
    feedback.box_name = box.name_;
    feedback.box_pose = box.pose_;
    box_detection_as_.publishFeedback(feedback);
}



}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "box_detection");

    try
    {
        tedusar_box_detection::BoxDetectionNode node;
        ros::spin();
        return 0;
    }
    catch (std::exception & ex)
    {
        std::cerr << "Unhandled exception: " << ex.what() << std::endl;
        return 1;
    }
}


