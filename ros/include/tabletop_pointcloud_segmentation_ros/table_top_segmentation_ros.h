/*
 * Maintainer: Mohammad Wasil
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef TABLETOP_POINTCLOUD_SEGMENTATION_ROS_H
#define TABLETOP_POINTCLOUD_SEGMENTATION_ROS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tabletop_pointcloud_segmentation_ros/clustered_point_cloud_visualizer.h>
#include <tabletop_pointcloud_segmentation_ros/bounding_box_visualizer.h>
#include <tabletop_pointcloud_segmentation_ros/label_visualizer.h>

#include <dynamic_reconfigure/server.h>
#include <tabletop_pointcloud_segmentation_ros/PlaneSegmentationConfig.h>
#include <string>
#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <tabletop_pointcloud_segmentation_ros/octree_pointcloud_occupancy_colored.h>
#include <tabletop_pointcloud_segmentation_ros/plane_segmentation.h>
#include <tabletop_pointcloud_segmentation_ros/color.h>
#include <common/bounding_box.h>
#include <perception_msgs/ImageCloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/pinhole_camera_model.h>

using mcr::visualization::BoundingBoxVisualizer;
using mcr::visualization::ClusteredPointCloudVisualizer;
using mcr::visualization::LabelVisualizer;
using mcr::visualization::Color;

/**
 * This node subscribes to pointcloud topic.
 * Inputs:
 * ~event_in:
 *      - e_start: starts subscribing to pointcloud topic
 *      - e_add_cloud_start: adds pointcloud to octree, if it is on dataset collection mode,
 *                           the node will start segmenting the pointcloud.
 *      - e_add_cloud_stop: stops adding pointcloud to octree
 *      - e_find_plane: finds the plane and publishes workspace height
 *      - e_segment: starts segmentation and publish ObjectList
 *      - e_reset: clears accumulated cloud
 *      - e_stop: stops subscribing and clears accumulated pointcloud
 * Outputs:
 * ~event_out:
 *      - e_started: started listening to new messages
 *      - e_add_cloud_started: started adding the cloud to octree
 *      - e_add_cloud_stopped: stopped adding the cloud to octree
 *      - e_done: started finding the plane or started segmenting the pointcloud
 *      - e_stopped: stopped subscribing and cleared accumulated pointcloud
 */

class TableTopSegmentationROS
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_debug_;
        ros::Publisher pub_debug_img_;
        ros::Publisher pub_boxes_;
        ros::Publisher pub_object_list_;
        ros::Publisher pub_event_out_;
        ros::Publisher pub_workspace_height_;
        ros::Publisher pub_input_for_debug_;
        ros::Publisher pub_image_roi_debug_;

        ros::Subscriber sub_cloud_;
        ros::Subscriber sub_image_;
        ros::Subscriber sub_event_in_;
        ros::Subscriber trigger_cam_sub_;
        ros::Subscriber sub_camera_info_;

        ros::ServiceClient recognize_service;
        std::string object_recognizer_service_name_;

        dynamic_reconfigure::Server<tabletop_pointcloud_segmentation_ros::PlaneSegmentationConfig> server_;

        tf::TransformListener transform_listener_;

        //SceneSegmentation scene_segmentation_;
        //PlaneSegmentation scene_segmentation_;
        //FlatPlaneSegmentation scene_segmentation_;

        //CloudAccumulation::UPtr cloud_accumulation_;

        BoundingBoxVisualizer bounding_box_visualizer_;
        ClusteredPointCloudVisualizer cluster_visualizer_;
        LabelVisualizer label_visualizer_;

        bool add_to_octree_;
        std::string frame_id_;
        int object_id_;
        double octree_resolution_;
        double object_height_above_workspace_;
        bool dataset_collection_;
        bool debug_mode_;
        std::string logdir_;

        bool trigger_camera_device_;
        //cv_bridge::CvImage cv_image_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        sensor_msgs::ImagePtr image_msg_;
        sensor_msgs::ImagePtr depth_msg_;
        cv_bridge::CvImagePtr cv_depth_;
        cv_bridge::CvImagePtr cv_image_;
        sensor_msgs::CameraInfoPtr camera_info_;

        message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> *cam_info_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> msgSyncPolicy;
        message_filters::Synchronizer<msgSyncPolicy> *msg_sync_;

        image_geometry::PinholeCameraModel cam_model_;
        sensor_msgs::CameraInfo info_msg_;

        std::string input_cloud_frame_id_;
        std::string class_name_;
    private:
        void pointcloudCallback(const perception_msgs::ImageCloudPtr &msg);
        void cameraInfoCallback(const sensor_msgs::CameraInfoPtr &msg);
        void triggerCameraCallback(const sensor_msgs::PointCloud2Ptr &msg);
        void TableTopSegmentationROS::imageCallback(const sensor_msgs::ImagePtr &msg);
        //void synchronizerCallback(const sensor_msgs::PointCloud2::Ptr &msg, const sensor_msgs::ImagePtr &img_msg);
        void eventCallback(const std_msgs::String::ConstPtr &msg);
        void configCallback(tabletop_pointcloud_segmentation_ros::PlaneSegmentationConfig &config, uint32_t level);
        void segment();
        void findPlane();
        geometry_msgs::PoseStamped getPose(const BoundingBox &box);
        void savePcd(const PointCloud::ConstPtr &cloud, const sensor_msgs::RegionOfInterest &roi);
        void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);
        void getPixelLocation(const Eigen::Vector3f &point);
        void sync_callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr &cloud);
        //void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg);
        

    public:
        TableTopSegmentationROS();
        virtual ~TableTopSegmentationROS();

    private:
        pcl::PassThrough<PointT> pass_through;
        pcl::VoxelGrid<PointT> voxel_grid;
        pcl::NormalEstimation<PointT, PointNT> normal_estimation;
        pcl::SACSegmentationFromNormals<PointT, PointNT> sac;
        pcl::ProjectInliers<PointT> project_inliers;
        pcl::ConvexHull<PointT> convex_hull;
        pcl::ExtractPolygonalPrismData<PointT> extract_polygonal_prism;
        pcl::EuclideanClusterExtraction<PointT> cluster_extraction;
        pcl::RadiusOutlierRemoval<PointT> radius_outlier;

    private:
        typedef OctreePointCloudOccupancyColored<PointT> Octree;
        typedef std::auto_ptr<Octree> OctreeUPtr;
        OctreeUPtr octree_;
        int cloud_count_;
        float resolution_ = 0.0025;

        void OctreeAddCloud(const PointCloud::ConstPtr& cloud);
        void OctreeGetAccumulatedCloud(PointCloud& cloud);
        void OctreeReset();

    private:
        //typedef std::auto_ptr<PlaneSegmentation> PlaneSegmentationUPtr;        
        //PlaneSegmentationUPtr plane_segmentation_;
        PlaneSegmentation plane_segmentation_;

};

#endif  // TABLETOP_POINTCLOUD_SEGMENTATION_ROS_H
