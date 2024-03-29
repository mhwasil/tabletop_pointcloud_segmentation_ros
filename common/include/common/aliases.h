/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 *
 */
#ifndef ALIASES_H
#define ALIASES_H

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/segmentation/planar_region.h>

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;
typedef pcl::Label PointLT;

typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<PointLT> PointCloudL;

typedef pcl::PlanarPolygon<PointT> PlanarPolygon;
typedef std::vector<PlanarPolygon, Eigen::aligned_allocator<PlanarPolygon> > PlanarPolygonVector;
typedef boost::shared_ptr<PlanarPolygon> PlanarPolygonPtr;
typedef boost::shared_ptr<const PlanarPolygon> PlanarPolygonConstPtr;

typedef pcl::PlanarRegion<PointT> PlanarRegion;
typedef std::vector<PlanarRegion, Eigen::aligned_allocator<PlanarRegion> > PlanarRegionVector;

#endif  // ALIASES_H
