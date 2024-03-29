/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka, Sergey Alexandrov
 *
 */
#ifndef TABLETOP_POINTCLOUD_SEGMENTATION_ROS_LABEL_VISUALIZER_H
#define TABLETOP_POINTCLOUD_SEGMENTATION_ROS_LABEL_VISUALIZER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <tabletop_pointcloud_segmentation_ros/color.h>

namespace mcr
{

namespace visualization
{

class LabelVisualizer
{
public:
    LabelVisualizer(const ros::NodeHandle &nh, const std::string& topic_name,
                          Color color,
                          bool check_subscribers = true);

    LabelVisualizer(const std::string& topic_name,
                          Color color,
                          bool check_subscribers = true);

    void publish(const std::vector<std::string> &labels, const geometry_msgs::PoseArray &poses);

    int getNumSubscribers();

private:
    ros::Publisher marker_publisher_;

    const Color color_;
    bool check_subscribers_;
};

}  // namespace visualization

}  // namespace mcr
#include "impl/label_visualizer.hpp"

#endif  // TABLETOP_POINTCLOUD_SEGMENTATION_ROS_LABEL_VISUALIZER_H
