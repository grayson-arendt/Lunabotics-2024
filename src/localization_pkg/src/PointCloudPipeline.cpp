#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include <pcl/common/common.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <string>
#include <thread>

/*
Author: Grayson Arendt

This program acts as a subscriber to camera/depth/color/points (which is the depth camera's point cloud data), 
uses PCL to apply various filters/algorithms, then streams the point cloud data to PCL visualizer. 
This is currently a work in progress, but I will add some comments in the meantime.
*/

using namespace std::chrono_literals;

// PCL visualizer 
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Viewer"));

class pointCloudPipeline : public rclcpp::Node {

    public: 

        // Camera subscriber
        pointCloudPipeline() : Node("pipeline"), previous_cloud_id_(""), previous_bbox_id_(""), filterType_("") {
            camera_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera/depth/color/points", 10, std::bind(&pointCloudPipeline::pipelineCallback, this, std::placeholders::_1));
        }

    private:

        // Callback for subscriber
        void pipelineCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points FILTER_TYPE: %s", msg->width * msg->height, filterType_.c_str());

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::fromROSMsg(*msg, *cloud);
            pcl::visualization::Camera camera;

            viewer->setBackgroundColor(0,0,0);

            viewer->getCameraParameters(camera);

            // Stream cloud to PCL visualizer
            streamPointCloud<pcl::PointNormal>(surfaceSmooth(passThroughFilter(outlierRemoval(voxelFilter(cloud, 0.07), 0.2, 10))));
            pcl::PointXYZRGB minPt, maxPt;

            //euclideanSeg(passThroughFilter(outlierRemoval(voxelFilter(cloud, 0.07), 0.2, 10)));
            pcl::getMinMax3D (*passThroughFilter(outlierRemoval(voxelFilter(cloud, 0.07), 0.2, 10)), minPt, maxPt);
            std::cout << "Max x: " << maxPt.x << std::endl;
            std::cout << "Max y: " << maxPt.y << std::endl;
            std::cout << "Max z: " << maxPt.z << std::endl;
            std::cout << "Min x: " << minPt.x << std::endl;
            std::cout << "Min y: " << minPt.y << std::endl;
            std::cout << "Min z: " << minPt.z << std::endl;

            viewer->spinOnce(100);
            std::this_thread::sleep_for(100ms);
        }

        // Surface smoothing algorithm (very laggy)
        const pcl::shared_ptr<pcl::PointCloud<pcl::PointNormal>> surfaceSmooth(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud) {

	        pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);
	        pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> filter;

            // Input cloud
	        filter.setInputCloud(cloud);

	        // Use all neighbors in a radius of 3cm.
	        filter.setSearchRadius(0.1);

	        // If true, the surface and normal are approximated using a polynomial estimation
	        // (if false, only a tangent one)
	        filter.setPolynomialOrder(3);

	        filter.setComputeNormals(true);

	        // kdtree object for searches
	        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree;
	        filter.setSearchMethod(kdtree);

	        filter.process(*smoothedCloud);
            return smoothedCloud;
        }

        // Voxel grid filter (downsampling point cloud data, each new point represents a "voxel" --a certain amount of space)
        const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> voxelFilter(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, float leaf_size) {
            filterType_ = "VOXEL-GRID";

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;

            // Input cloud
            voxelGrid.setInputCloud(cloud);

            // Leaf size (side length of each "voxel")
            voxelGrid.setLeafSize(leaf_size,leaf_size,leaf_size);

            voxelGrid.filter(*filteredCloud);
            return filteredCloud;
        }

        // Pass-through filter (removes points after a certain range in the specified axis)
        const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> passThroughFilter(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud) {
            filterType_ = "PASS-THROUGH";

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	        pcl::PassThrough<pcl::PointXYZRGB> filter;

            // Input cloud
	        filter.setInputCloud(cloud);

	        // Filter out all points with Z values not in the [0-2] range.
	        filter.setFilterFieldName("y");
	        filter.setFilterLimits(0.0, 3.0);

	        filter.filter(*filteredCloud);
            return filteredCloud;
        }

        // Outlier removal (important for cleaning initial point cloud)
        const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> outlierRemoval(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, double radius, int points) {

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> filter;

            // Input cloud
	        filter.setInputCloud(cloud);

	        // Every point must have 10 neighbors within 10cm, or it will be removed
	        filter.setRadiusSearch(radius);
	        filter.setMinNeighborsInRadius(points);

	        filter.filter(*filteredCloud);
            return filteredCloud;
        }

        // Regional segmentation (similar to euclidean but takes into account the angles of each point)
        void regionSeg(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud) {

            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
            pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> clustering;
            std::vector<pcl::PointIndices> clusters;

            // kdtree for searches
	        kdtree->setInputCloud(cloud);
            
	        // Input cloud
	        normalEstimation.setInputCloud(cloud);

            // Search neighbors and compute normals
	        normalEstimation.setRadiusSearch(0.03);
	        normalEstimation.setSearchMethod(kdtree);
	        normalEstimation.compute(*normals);

	        // Clusters for each region detected
	        clustering.setMinClusterSize(50);
	        clustering.setMaxClusterSize(10000);
	        clustering.setSearchMethod(kdtree);
	        clustering.setNumberOfNeighbours(30);
	        clustering.setInputCloud(cloud);
	        clustering.setInputNormals(normals);

	        // Set the angle in radians that will be the smoothness threshold (the maximum allowable deviation of the normals)
	        clustering.setSmoothnessThreshold(7.0 / 180.0 * M_PI); // 7 degrees

	        // Set the curvature threshold
	        clustering.setCurvatureThreshold(1.0);

            clustering.extract(clusters);
            streamClusters(clusters, cloud);
        }

        // RANSAC segmentation (getting largest plane of points)
        const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> ransacSeg(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud){
            filterType_ = "RANSAC";

            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;


             // RANSAC segmentation to find the main plane (cluster)
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.05);
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);      

            // Extract the main cluster points
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);

            extract.filter(*mainCluster);
            return mainCluster;
        }

        // Euclidean clustering
        void euclideanSeg(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud) {
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
            std::vector<pcl::PointIndices> clusters;
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            kdtree->setInputCloud(cloud);

            // Identifying clusters of points
            clustering.setClusterTolerance(0.075); // 8 cm, this value is very sensitive and needs fine-tuning depending on environment
            clustering.setMinClusterSize(100);
            clustering.setMaxClusterSize(25000);
            clustering.setSearchMethod(kdtree);
            clustering.setInputCloud(cloud);
            
            // Extract clusters
            clustering.extract(clusters);
            streamClusters(clusters, cloud);
        }

        // Streaming clusters (makes each point of new cluster a different color for visualization purposes)
        void streamClusters(std::vector<pcl::PointIndices> clusters, const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud) {
            int numberOfClusters = 0;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            // For each cluster in extracted clusters
            for (const auto& cluster : clusters) {
                // Set color values
                int r = std::rand() % 256;
                int g = std::rand() % 256;
                int b = std::rand() % 256;
             
                // For each point inside the cluster, add it to a new point cloud (cluster_cloud) as a certain color
                for (auto index : cluster.indices) {
                    auto& point = cloud->at(index);

                    pcl::PointXYZRGB newPoint;
                    newPoint.x = point.x;
                    newPoint.y = point.y;
                    newPoint.z = point.z;
                    newPoint.r = r;    
                    newPoint.g = g;  
                    newPoint.b = b;   

                    // Adding to cloud
                    cluster_cloud->push_back(newPoint);
                } 

                // Streams the new cluster point cloud
                streamPointCloud<pcl::PointXYZRGB>(cluster_cloud);

                //createBoundingBox(cluster_cloud);

                numberOfClusters++; 
            }

            // Output cluster size to terminal
            RCLCPP_INFO(this->get_logger(), "Cluster size: %i", numberOfClusters);
        }

        // Generates a bounding box around a point cloud
        void createBoundingBox(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud) {
     
            //Compute centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud, centroid);

            //Compute covariance matrix
            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covariance, Eigen::ComputeEigenvectors);

            //Get eigenvectors
            Eigen::Matrix3f eigenVectors = eigenSolver.eigenvectors();

            //Correcting orientation of bounding box 
            eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));

            //Transform pointcloud to origin
            Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
            projectionTransform.block<3,3>(0,0) = eigenVectors.transpose();
            projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * centroid.head<3>());

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);            

            //Minimum and maximum points
            pcl::PointXYZRGB minPoint, maxPoint;
            pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
            const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

            //Last transform
            const Eigen::Quaternionf bboxQuaternion(eigenVectors);
            const Eigen::Vector3f bboxTransform = eigenVectors * meanDiagonal + centroid.head<3>();

            // Generate a new id
            std::string bbox_id = "bbox_" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

            // Remove previous box (to only have one box show at a time for each new input point cloud)
            if(!previous_bbox_id_.empty()) {
                viewer->removeShape(previous_bbox_id_);
            }

            // Adds cube to PCL visualizer
            viewer->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, bbox_id);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, bbox_id);

            previous_bbox_id_ = bbox_id;
        }

        template <typename PointCloudType> 

        // Streams the point cloud (uses template due to only having two types of used point clouds in this class)
        void streamPointCloud(const typename pcl::PointCloud<PointCloudType>::Ptr& cloud) {
      
            // Generate a new id 
            std::string cloud_id = "cloud_" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

            //Remove previous pointcloud 
            if(!previous_cloud_id_.empty()) {
                viewer->removePointCloud(previous_cloud_id_);
            }

            // Checking if it's either normal or XYZRGB (since surface smoothing is of normal type and the rest of the methods are XYZRGB)
            if(std::is_same<PointCloudType, pcl::PointXYZRGB>::value || std::is_same<PointCloudType, pcl::PointNormal>::value) {

                // Add point cloud to viewer
                viewer->addPointCloud<PointCloudType>(cloud, cloud_id);
            } 

            else {
                
                // Error message if not normal or XYZRGB
                std::string error_msg = "streamPointCloud() error: unsupported point cloud type";
                RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
            }

            previous_cloud_id_ = cloud_id;
            //createBoundingBox(cloud);
        }

    // Variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_sub_;
    std::string previous_cloud_id_;
    std::string previous_bbox_id_;
    std::string filterType_;

};

int main(int argc, char **argv) {

    viewer->setCameraPosition(0, -1, -6,  // Camera position (x, y, z)
                              0, 0, 0,  // Focal point (x, y, z) - set to the center of the point cloud
                              0.3, -1, 0.5);  // View up vector (x, y, z) - set to (0, -1, 0) to flip the camera orientation


    // Start ROS2
    rclcpp::init(argc, argv);
 
    // Keep running node
    rclcpp::spin(std::make_shared<pointCloudPipeline>());

    // Stop ROS2
    rclcpp::shutdown();

    return 0;
}