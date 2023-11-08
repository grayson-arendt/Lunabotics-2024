#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <string>

/*
Author: Grayson Arendt

This class provides methods for filtering and condensing a PointCloud from 3D into 2D format
to allow for usage of 2D SLAM. Ideally, this class would be used with a depth camera to track
objects above a certain range (with pass-through filter) and objects below a certain range, then
the two PointClouds can be merged together and converted to a ROS2 LaserScan message to simulate
the Lidar data that is used in most 2D SLAM applications.
*/

class PointCloudTo2D {

public:

    // Voxel grid filter (down sampling point cloud data, each new point represents a "voxel" --a certain amount of space)
    static pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> voxelFilter(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud, float leaf_size) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;

        voxelGrid.setInputCloud(cloud);

        // Leaf size (side length of each "voxel")
        voxelGrid.setLeafSize(leaf_size,leaf_size,leaf_size);

        voxelGrid.filter(*filteredCloud);
        return filteredCloud;
    }

    // Outlier removal (removes excess points outside certain radius)
    static pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> outlierRemoval(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud, double radius, int min_points) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> filter;

        filter.setInputCloud(cloud);

        // Every point must have 10 neighbors within 10cm, or it will be removed
        filter.setRadiusSearch(radius);
        filter.setMinNeighborsInRadius(min_points);

        filter.filter(*filteredCloud);
        return filteredCloud;
    }

    // RANSAC plane segmentation
    static pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> planeSegmentation(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud, double threshold) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundPlane(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);

        seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(500);
        seg.setDistanceThreshold(threshold);

        Eigen::Vector3f axis = Eigen::Vector3f(1.0, 0.0, 1.0); //y axis
        seg.setAxis(axis);
        seg.setEpsAngle(10.0 * (M_PI / 180.0f)); // plane can be within 10.0 degrees of X-Z plane

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;

        // Extracting calculated indices
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*groundPlane);

        return groundPlane;
    }

    // Pass-through filter (removes points after a certain range in the specified axis)
    static pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> passThroughFilter(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud, float min, float max, const std::string& axis) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> filter;

        filter.setInputCloud(cloud);

        // Filter out all points with Z values not in the [0-2] range.
        filter.setFilterFieldName(axis);
        filter.setFilterLimits(min, max);

        filter.filter(*filteredCloud);
        return filteredCloud;
    }

    /* Parameters:
     * leaf_size for voxel grid filter
     * radius, min_points for outlier removal
     * min, max, and axis for pass-through filter
    */

    static pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> condensePointCloud(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr condensedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // For each point in PointCloud, extract x and z values (z measures depth, so it is 2D y equivalent)
        for (const auto &point : cloud->points) {
            pcl::PointXYZRGB condensedPoint;

            condensedPoint.x = point.z;
            condensedPoint.y = -point.x;
            condensedPoint.z = 0.0;

            condensedCloud->push_back(condensedPoint);

        }

        return condensedCloud;
    }
};
