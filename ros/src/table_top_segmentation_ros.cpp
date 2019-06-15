/*
 * Maintainer: Mohammad Wasil
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */

#include <perception_msgs/BoundingBox.h>
#include <perception_msgs/BoundingBoxList.h>
#include <perception_msgs/ObjectList.h>
#include <perception_msgs/RecognizeObject.h>
#include <tabletop_pointcloud_segmentation_ros/impl/helpers.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <tabletop_pointcloud_segmentation_ros/table_top_segmentation_ros.h>
#include <tabletop_pointcloud_segmentation_ros/octree_pointcloud_occupancy_colored.h>
#include <pcl/octree/octree_impl.h>
#include <tabletop_pointcloud_segmentation_ros/plane_segmentation.h>
#include <perception_msgs/ImageCloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


TableTopSegmentationROS::TableTopSegmentationROS(): nh_("~"),cluster_visualizer_("tabletop_clusters"),
    bounding_box_visualizer_("bounding_boxes", Color(Color::SEA_GREEN)),
    label_visualizer_("labels", mcr::visualization::Color(mcr::visualization::Color::TEAL)),
    add_to_octree_(false), object_id_(0), debug_mode_(false), dataset_collection_(false),
    trigger_camera_device_(false)
{
    pub_debug_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 1);
    pub_object_list_ = nh_.advertise<perception_msgs::ObjectList>("object_list", 1);
    sub_event_in_ = nh_.subscribe("event_in", 1, &TableTopSegmentationROS::eventCallback, this);
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
    pub_workspace_height_ = nh_.advertise<std_msgs::Float64>("workspace_height", 1);
    pub_image_roi_debug_ = nh_.advertise<sensor_msgs::Image>("output/image_roi_debug", 1);

    dynamic_reconfigure::Server<tabletop_pointcloud_segmentation_ros::PlaneSegmentationConfig>::CallbackType f =
                            boost::bind(&TableTopSegmentationROS::configCallback, this, _1, _2);
    
    server_.setCallback(f);

    nh_.param("octree_resolution", octree_resolution_, 0.05);

    nh_.param<bool>("debug_mode", debug_mode_, "false");
    nh_.param<bool>("dataset_collection", dataset_collection_, "false");
    nh_.param<std::string>("logdir", logdir_, "/tmp/");

    octree_ = OctreeUPtr(new Octree(0.0025));
    
    pub_debug_img_ = nh_.advertise<sensor_msgs::Image>("image_output", 1);

    // if (!trigger_camera_device_)
    // {   
    ROS_INFO_STREAM("Triggering camera device...");
    trigger_cam_sub_ = nh_.subscribe("/pico_flexx/points", 1, &TableTopSegmentationROS::triggerCameraCallback, this);

    sub_camera_info_ = nh_.subscribe("/arm_cam3d/depth/camera_info", 1, &TableTopSegmentationROS::cameraInfoCallback, this);
    camera_info_ = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);

    nh_.param<std::string>("class_name", class_name_, "object");

}
    

TableTopSegmentationROS::~TableTopSegmentationROS()
{
}

void TableTopSegmentationROS::sync_callback(const sensor_msgs::ImageConstPtr& image, 
                                            const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    std::cout<<"===================="<<std::endl;
    std::cout<<"Message got sycronized"<<std::endl;
    std::cout<<image->header.stamp<<", "<<cloud->header.stamp<<std::endl;
    input_cloud_frame_id_ = cloud->header.frame_id;
    std::string target_frame_id;
    nh_.param<std::string>("target_frame_id", target_frame_id, "base_link");
    sensor_msgs::PointCloud2 msg_transformed;
    msg_transformed.header.frame_id = target_frame_id;
    std::cout<<"Frame id: "<<cloud->header.frame_id<<std::endl;
    try
    {
        ros::Time common_time;
        transform_listener_.getLatestCommonTime(target_frame_id, cloud->header.frame_id, common_time, NULL);
        cloud->header.stamp = common_time;
        transform_listener_.waitForTransform(target_frame_id, cloud->header.frame_id,
                                                ros::Time::now(), ros::Duration(1.0));
        pcl_ros::transformPointCloud(target_frame_id, *cloud, msg_transformed, transform_listener_);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("PCL transform error: %s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    frame_id_ = msg_transformed.header.frame_id;
    //PointCloud::Ptr cloud(new PointCloud);
    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pc2;
    pcl_conversions::toPCL(msg_transformed, pc2);
    pcl::fromPCLPointCloud2(pc2, *cloud_);
    cloud_->header.frame_id = target_frame_id;
    std::cout<<"Cloud size"<<std::endl;
    std::cout<<cloud_->height<<", "<<cloud_->width<<std::endl;

    try
    {
        cv_image_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    segment();

    image_sub_->unsubscribe();
    cloud_sub_->unsubscribe();
}

void TableTopSegmentationROS::OctreeAddCloud(const PointCloud::ConstPtr& cloud)
{
    octree_->setOccupiedVoxelsAtPointsFromCloud(cloud);
    //cloud_count_++;
}

void TableTopSegmentationROS::OctreeGetAccumulatedCloud(PointCloud& cloud)
{
    octree_->getOccupiedVoxelCentersWithColor(cloud.points);
    cloud.width = cloud.points.size();
    cloud.height = 1;
}

void TableTopSegmentationROS::OctreeReset()
{
    octree_ = OctreeUPtr(new Octree(resolution_));
    //cloud_count_ = 0;
}

void TableTopSegmentationROS::triggerCameraCallback(const sensor_msgs::PointCloud2Ptr& msg)
{
    trigger_camera_device_ = true;
}

void TableTopSegmentationROS::cameraInfoCallback(const sensor_msgs::CameraInfoPtr& msg)
{
    //camera_info_ = msg;
    info_msg_ = *msg;
    cam_model_.fromCameraInfo(info_msg_);
}

// Use pinhole camera params to find 2d pixel location
void TableTopSegmentationROS::getPixelLocation(const Eigen::Vector3f &point)
{
    int x, y;
    x = camera_info_->K[0]*(point[0]/point[2]) + camera_info_->K[2];
    y = camera_info_->K[4]*(point[1]/point[2]) + camera_info_->K[5];
    std::cout<<"X,Y: "<<x<<", "<<y<<std::endl;
}

void TableTopSegmentationROS::pointcloudCallback(const perception_msgs::ImageCloudPtr &msg)
{
    std::string target_frame_id;
    nh_.param<std::string>("target_frame_id", target_frame_id, "base_link");
    sensor_msgs::PointCloud2 msg_transformed;
    msg_transformed.header.frame_id = target_frame_id;
    std::cout<<"Frame id: "<<msg->pointcloud.header.frame_id<<std::endl;
    try
    {
        ros::Time common_time;
        transform_listener_.getLatestCommonTime(target_frame_id, msg->pointcloud.header.frame_id, common_time, NULL);
        msg->pointcloud.header.stamp = common_time;
        transform_listener_.waitForTransform(target_frame_id, msg->pointcloud.header.frame_id,
                                                ros::Time::now(), ros::Duration(1.0));
        pcl_ros::transformPointCloud(target_frame_id, msg->pointcloud, msg_transformed, transform_listener_);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("PCL transform error: %s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    //PointCloud::Ptr cloud(new PointCloud);
    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pc2;
    pcl_conversions::toPCL(msg_transformed, pc2);
    pcl::fromPCLPointCloud2(pc2, *cloud_);
    
    OctreeAddCloud(cloud_);

    frame_id_ = msg_transformed.header.frame_id;
    
    ros::Time end_time = ros::Time::now();

    // Convert to cv image
    image_msg_ = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    *image_msg_ = msg->image;
    cv_image_ = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::MONO8);
    std::cout<<"Image size: "<<cv_image_->image.size()<<std::endl;

    // Get depth and convert it to cv image
    depth_msg_ = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    *depth_msg_ = msg->depth;
    cv_depth_ = cv_bridge::toCvCopy(depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
    
    pub_debug_img_.publish(cv_depth_->toImageMsg());

    segment();
    OctreeReset();

    std_msgs::String event_out;
    event_out.data = "e_saved";
    pub_event_out_.publish(event_out);

    sub_cloud_.shutdown();

}

//https://answers.ros.org/question/60182/error-unable-to-convert-32fc1-image-to-bgr8-while-extracting-rbgd-data-from-a-bag/
void TableTopSegmentationROS::depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img)
{
  //Process images
    if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols)
    {
        mono8_img = cv::Mat(float_img.size(), CV_8UC1);
    }
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
  //The following doesn't work due to NaNs
  //double minVal, maxVal; 
  //minMaxLoc(float_img, &minVal, &maxVal);
  //ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  //mono8_img = cv::Scalar(0);
  //cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8);
}

void TableTopSegmentationROS::segment()
{
    std::vector<PointCloud::Ptr> clusters;
    std::vector<BoundingBox> boxes;
    double workspace_height;
    PointCloud::Ptr debug = plane_segmentation_.segment_scene(cloud_, clusters, boxes, workspace_height);
    
    debug->header.frame_id = cloud_->header.frame_id;
    std_msgs::Float64 workspace_height_msg;
    workspace_height_msg.data = workspace_height;
    pub_workspace_height_.publish(workspace_height_msg);
    pub_debug_.publish(*debug);

    perception_msgs::BoundingBoxList bounding_boxes;
    perception_msgs::ObjectList object_list;
    geometry_msgs::PoseArray poses;

    bounding_boxes.bounding_boxes.resize(boxes.size());
    object_list.objects.resize(boxes.size());

    std::vector<std::string> labels;

    ros::Time now = ros::Time::now();
    for (int i = 0; i < boxes.size(); i++)
    {
        convertBoundingBox(boxes[i], bounding_boxes.bounding_boxes[i]);

        sensor_msgs::PointCloud2 ros_cloud;
        pcl::PCLPointCloud2 pc2;
        pcl::toPCLPointCloud2(*clusters[i], pc2);
        pcl_conversions::fromPCL(pc2, ros_cloud);

        BoundingBox::Points vertices = boxes[i].getVertices();

        sensor_msgs::PointCloud2 ros_cloud_transformed;
        ros_cloud_transformed.header.frame_id = input_cloud_frame_id_;
        try
        {
            ros::Time common_time;
            transform_listener_.getLatestCommonTime(input_cloud_frame_id_, ros_cloud.header.frame_id, common_time, NULL);
            ros_cloud.header.stamp = common_time;
            transform_listener_.waitForTransform(input_cloud_frame_id_, ros_cloud.header.frame_id,
                                                    ros::Time::now(), ros::Duration(1.0));
            pcl_ros::transformPointCloud(input_cloud_frame_id_, ros_cloud, ros_cloud_transformed, transform_listener_);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("PCL transform error: %s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

        pcl::PointCloud<PointT>::Ptr pointcloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(ros_cloud_transformed, *pointcloud);
        int cloud_size = pointcloud->points.size();
        std::cout<<"Cloud size: "<<cloud_size<<std::endl;
        std::cout<<pointcloud->height<<", "<<pointcloud->width<<std::endl;

        pcl::PointXYZ min_pt;
        pcl::PointXYZ max_pt;
        pcl::getMinMax3D(*pointcloud, min_pt, max_pt);
        std::cout<<"Min max x,y: "<<std::endl;
        std::cout<<min_pt.x<<", "<<max_pt.x<<std::endl;
        std::cout<<min_pt.y<<", "<<max_pt.y<<std::endl;

        std::cout<<info_msg_.K[0]<<std::endl;
        double min_px = (min_pt.x * info_msg_.K[0])/min_pt.z + info_msg_.K[2];
        double min_py = (min_pt.y * info_msg_.K[4])/min_pt.z + info_msg_.K[5];
        double max_px = (max_pt.x * info_msg_.K[0])/max_pt.z + info_msg_.K[2];
        double max_py = (max_pt.y * info_msg_.K[4])/max_pt.z + info_msg_.K[5];

        int tolerance = 5;
        if (min_px - tolerance >= 0) min_px = min_px - tolerance;
        if (min_py - tolerance >= 0) min_py = min_py - tolerance;
        if (max_px + tolerance <= cv_image_->image.rows) max_px = max_px + tolerance;
        if (max_py + tolerance <= cv_image_->image.cols) max_py = max_py + tolerance;

        std::cout<<"Min Pixel loc "<<std::endl;
        std::cout<<"Min: "<<min_px<<", "<<min_py<<std::endl;
        std::cout<<"Max: "<<max_px<<", "<<max_py<<std::endl;

        int x = static_cast<int>(min_px);
        int y = static_cast<int>(min_py);
        int width = static_cast<int>(max_px) - x;
        int height = static_cast<int>(max_py) - y;

        sensor_msgs::RegionOfInterest roi;
        roi.x_offset = x;
        roi.y_offset = y;
        roi.width = width;
        roi.height = height;

        std::cout<<"ROI: "<<std::endl;
        std::cout<<x<<", "<<width<<std::endl;
        std::cout<<y<<", "<<height<<std::endl;

        cv::Rect rect(x, y, width, height);
        cv::rectangle(cv_image_->image, rect, cv::Scalar(255, 0, 0));

        //cv_bridge::CvImagePtr cv_ptr;
        pub_image_roi_debug_.publish(cv_image_->toImageMsg());

        if (cloud_size >= 3)
        {
            cloud_count_ += 1;
            TableTopSegmentationROS::savePcd(pointcloud, roi);
        }
        else
        {
            ROS_INFO_STREAM("Not enough points to save (<3)");
        }
    }
    pub_object_list_.publish(object_list);
    bounding_box_visualizer_.publish(bounding_boxes.bounding_boxes, frame_id_);
    cluster_visualizer_.publish<PointT>(clusters, frame_id_);
    label_visualizer_.publish(labels, poses);

}

void TableTopSegmentationROS::savePcd(const PointCloud::ConstPtr &pointcloud, const sensor_msgs::RegionOfInterest &roi)
{
    std::stringstream filename; // stringstream used for the conversion
    ros::Time time_now = ros::Time::now();

    //create text info
    filename.str("");
    filename << logdir_ <<class_name_<<"_text_" << time_now <<".txt";
    std::ofstream outfile(filename.str().c_str());

    filename.str(""); //clearing the stringstream
    filename << logdir_ <<class_name_<<"_pcd_" << time_now <<".pcd";
    ROS_INFO_STREAM("Saving pointcloud to " << logdir_);
    pcl::io::savePCDFileASCII (filename.str(), *pointcloud);
    filename.str(""); //clearing the stringstream
    filename <<class_name_<<"_pcd_" << time_now <<".pcd";
    outfile <<"pcd: "<<filename.str()<< std::endl;
    //std::cout<<"File name: "<<filename.str()<<std::endl;

    // Save gray image
    // filename.str("");
    // filename << logdir_ <<"gray_" << time_now <<".png";
    // cv::imwrite(filename.str(), cv_image_->image);

    // Save depth image
    // Convert 32FC1 to mono8
    // cv::Mat depth_float_img = cv_depth_->image;
    // cv::Mat depth_mono8_img;
    // depthToCV8UC1(depth_float_img, depth_mono8_img);
    // filename.str("");
    // filename << logdir_ <<"depth_" << time_now <<".png";
    // cv::imwrite(filename.str(), depth_mono8_img);

    // Save image and roi
    double min_px = roi.x_offset;
    double min_py = roi.y_offset;
    double max_px = roi.x_offset + roi.width;
    double max_py = roi.y_offset + roi.height;
    filename.str("");
    filename << logdir_ <<class_name_<<"_rgb_" << time_now <<".jpg";
    cv::imwrite(filename.str(), cv_image_->image);
    filename.str(""); 
    filename <<class_name_<<"_rgb_" << time_now <<".jpg";
    outfile <<"rgb: "<<filename.str()<< std::endl;
    
    outfile <<"bbox: " <<min_px <<" "<< min_py << " " << max_px<< " " << max_py << std::endl;
    outfile.close();

}

void TableTopSegmentationROS::findPlane()
{
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = frame_id_;
    OctreeGetAccumulatedCloud(*cloud);

    double workspace_height;
    PointCloud::Ptr hull(new PointCloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    PointCloud::Ptr debug = plane_segmentation_.findPlane(cloud, hull, coefficients, workspace_height);
    debug->header.frame_id = cloud->header.frame_id;
    std_msgs::Float64 workspace_height_msg;
    workspace_height_msg.data = workspace_height;
    pub_workspace_height_.publish(workspace_height_msg);
    // /pub_debug_.publish(*debug);
}

geometry_msgs::PoseStamped TableTopSegmentationROS::getPose(const BoundingBox &box)
{
    BoundingBox::Points vertices = box.getVertices();
    Eigen::Vector3f n1;
    Eigen::Vector3f n2;
    Eigen::Vector3f n3 = (vertices[4] - vertices[0]) / (vertices[4] - vertices[0]).norm();
    if ((vertices[1] - vertices[0]).norm() > (vertices[3] - vertices[0]).norm())
    {
        n1 = (vertices[1] - vertices[0]) / (vertices[1] - vertices[0]).norm();
    }
    else
    {
        n1 = (vertices[3] - vertices[0]) / (vertices[3] - vertices[0]).norm();
    }
    n2 = n3.cross(n1);
    ROS_INFO_STREAM("got norms");
    Eigen::Matrix3f m;
    m << n1 , n2 , n3;
    Eigen::Quaternion<float> q(m);
    q.normalize();

    double workspace_height = (vertices[0](2) + vertices[1](2) + vertices[2](2) + vertices[3](2)) / 4.0;

    Eigen::Vector3f centroid = box.getCenter();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = centroid(0);
    pose.pose.position.y = centroid(1);
    pose.pose.position.z = workspace_height + object_height_above_workspace_;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
}

void TableTopSegmentationROS::eventCallback(const std_msgs::String::ConstPtr &msg)
{
    std_msgs::String event_out;
    if (msg->data == "e_start")
    {   
        cloud_count_ = 0;        
        image_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh_, "/arm_cam3d/rgb/image_raw", 1);
        cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "/arm_cam3d/depth_registered/points", 1);
        //cam_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo> (nh_, "/arm_cam3d/depth/camera_info", 1);
        msg_sync_ = new message_filters::Synchronizer<msgSyncPolicy> (msgSyncPolicy(10), *image_sub_, *cloud_sub_);
        msg_sync_->registerCallback(boost::bind(&TableTopSegmentationROS::sync_callback, this, _1, _2));
        event_out.data = "e_started";
        
        //trigger_cam_sub_.shutdown();
        //sub_camera_info_.shutdown();
    }
    pub_event_out_.publish(event_out);
}

void TableTopSegmentationROS::configCallback(tabletop_pointcloud_segmentation_ros::PlaneSegmentationConfig &config, uint32_t level)
{
    plane_segmentation_.setVoxelGridParams(config.voxel_leaf_size, config.voxel_filter_field_name,
            config.voxel_filter_limit_min, config.voxel_filter_limit_max);
    plane_segmentation_.setPassthroughParams(config.passthrough_filter_field_name,
            config.passthrough_filter_limit_min,
            config.passthrough_filter_limit_max);
    plane_segmentation_.setNormalParams(config.normal_radius_search);
    plane_segmentation_.setSACParams(config.sac_max_iterations, config.sac_distance_threshold,
            config.sac_optimize_coefficients, config.sac_eps_angle,
            config.sac_normal_distance_weight);
    plane_segmentation_.setPrismParams(config.prism_min_height, config.prism_max_height);
    plane_segmentation_.setOutlierParams(config.outlier_radius_search, config.outlier_min_neighbors);
    plane_segmentation_.setClusterParams(config.cluster_tolerance, config.cluster_min_size, config.cluster_max_size,
            config.cluster_min_height, config.cluster_max_height, config.cluster_max_length,
            config.cluster_min_distance_to_polygon);
    object_height_above_workspace_ = config.object_height_above_workspace;
    octree_resolution_ = config.octree_resolution;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "table_top_segmentation");
    TableTopSegmentationROS scene_seg;
    ros::spin();
    return 0;
}

