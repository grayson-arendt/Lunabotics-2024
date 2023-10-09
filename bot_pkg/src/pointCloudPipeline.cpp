#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/search/kdtree.h"
#include "pcl/common/impl/centroid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "pcl/common/transforms.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include "pcl/io/ply_io.h"
#include "pcl/point_types.h"
#include <pcl/common/common.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <string>
#include <typeinfo>
#include <thread>

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Viewer"));

class pointCloudPipeline : public rclcpp::Node {
    public: 

        pointCloudPipeline() : Node("pipeline"), previous_cloud_id_(""), previous_bbox_id_(""), filterType_("") {
            camera_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera/depth/color/points", 10, std::bind(&pointCloudPipeline::pipelineCallback, this, std::placeholders::_1));
        }

    private:
  
        void pipelineCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points FILTER_TYPE: %s", msg->width * msg->height, filterType_.c_str());

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::fromROSMsg(*msg, *cloud);

            viewer->setBackgroundColor(0,0,0);
            
            //pcl::PointCloud<pcl::PointXYZRGB>::Ptr processedCloud = voxelFilter(outlierRemoval(cloud),0.05);
          
            streamPointCloud<pcl::PointXYZRGB>(outlierRemoval(cloud));
            //euclideanSeg(processedCloud);
        
            //euclideanSeg(voxelFilter(outlierRemoval(cloud),0.05));

            //streamPointCloud(passThroughFilter(outlierRemoval(cloud)));
            //euclideanSeg(voxelFilter(passThroughFilter(cloud)));
            
            viewer->spinOnce(100);
            std::this_thread::sleep_for(100ms);
        }

        const pcl::shared_ptr<pcl::PointCloud<pcl::PointNormal>> surfaceSmooth(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud) {
           	// The output will also contain the normals.
	        pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);

	        // Smoothing object (we choose what point types we want as input and output).
	        pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> filter;
	        filter.setInputCloud(cloud);
	        // Use all neighbors in a radius of 3cm.
	        filter.setSearchRadius(0.03);
	        // If true, the surface and normal are approximated using a polynomial estimation
	        // (if false, only a tangent one).
	        filter.setPolynomialOrder(2); 
	        // We can tell the algorithm to also compute smoothed normals (optional).
	        filter.setComputeNormals(true);
	        // kd-tree object for performing searches.
	        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree;
	        filter.setSearchMethod(kdtree);

	        filter.process(*smoothedCloud);
            return smoothedCloud;
        }


        const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> voxelFilter(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, float leaf_size) {
            filterType_ = "VOXEL-GRID";
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            //Filtering grid 
            pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
            voxelGrid.setInputCloud(cloud);
            voxelGrid.setLeafSize(leaf_size,leaf_size,leaf_size);
            voxelGrid.filter(*filteredCloud);
            return filteredCloud;
        }

        const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> passThroughFilter(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud) {
            filterType_ = "PASS-THROUGH";
            
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	        //Filter object.
	        pcl::PassThrough<pcl::PointXYZRGB> filter;
	        filter.setInputCloud(cloud);

	        // Filter out all points with Z values not in the [0-2] range.
	        filter.setFilterFieldName("z");
	        filter.setFilterLimits(0.0, 4.0); 

	        filter.filter(*filteredCloud);

            return filteredCloud;
        }

        const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> outlierRemoval(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	        // Filter object.
	        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> filter;
	        filter.setInputCloud(cloud);
	        // Every point must have 10 neighbors within 10cm, or it will be removed.
	        filter.setRadiusSearch(0.1);
	        filter.setMinNeighborsInRadius(100);

	        filter.filter(*filteredCloud);
            return filteredCloud;
        }

        void regionSeg(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud){
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
          
	        // kd-tree object for searches.
	        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	        kdtree->setInputCloud(cloud);

	        // Estimate the normals.
	        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	        normalEstimation.setInputCloud(cloud);
	        normalEstimation.setRadiusSearch(0.03);
	        normalEstimation.setSearchMethod(kdtree);
	        normalEstimation.compute(*normals);

	        // Region growing clustering object.
	        pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> clustering;
	        clustering.setMinClusterSize(50);
	        clustering.setMaxClusterSize(10000);
	        clustering.setSearchMethod(kdtree);
	        clustering.setNumberOfNeighbours(30);
	        clustering.setInputCloud(cloud);
	        clustering.setInputNormals(normals);
	        // Set the angle in radians that will be the smoothness threshold
	        // (the maximum allowable deviation of the normals).
	        clustering.setSmoothnessThreshold(7.0 / 180.0 * M_PI); // 7 degrees.
	        // Set the curvature threshold. The disparity between curvatures will be
	        // tested after the normal deviation check has passed.
	        clustering.setCurvatureThreshold(1.0);

            std::vector<pcl::PointIndices> clusters;
            clustering.extract(clusters);
            streamClusters(clusters, cloud);

        }

        const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> ransacSeg(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud){
            filterType_ = "RANSAC";
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCluster (new pcl::PointCloud<pcl::PointXYZRGB>);


             // RANSAC segmentation to find the main plane (cluster)
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.05);
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);      

            // Extract the main cluster points
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*mainCluster);
            return mainCluster;
        }

        void euclideanSeg(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud) {
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            kdtree->setInputCloud(cloud);

            //Euclidean clustering
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
            clustering.setClusterTolerance(0.08);
            clustering.setMinClusterSize(100);
            clustering.setMaxClusterSize(25000);
            clustering.setSearchMethod(kdtree);
            clustering.setInputCloud(cloud);
            
            //Extract clusters
            std::vector<pcl::PointIndices> clusters;
            clustering.extract(clusters);
            streamClusters(clusters, cloud);
        }

        void streamClusters(std::vector<pcl::PointIndices> clusters, const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud) {
            int numberOfClusters = 0;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            for (const auto& cluster : clusters) {
                int r = std::rand() % 256;
                int g = std::rand() % 256;
                int b = std::rand() % 256;
             
                for (auto index : cluster.indices) {
                    auto& point = cloud->at(index);

                    pcl::PointXYZRGB newPoint;
                    newPoint.x = point.x;
                    newPoint.y = point.y;
                    newPoint.z = point.z;
                    newPoint.r = r;    
                    newPoint.g = g;  
                    newPoint.b = b;   

                    cluster_cloud->push_back(newPoint);   
            
                    } //*

                streamPointCloud<pcl::PointXYZRGB>(cluster_cloud);
                //createBoundingBox(cluster_cloud);

                numberOfClusters++; 
            }

            RCLCPP_INFO(this->get_logger(), "Cluster size: %i", numberOfClusters);
        }

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


            std::string bbox_id = "bbox_" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

            if(!previous_bbox_id_.empty()) {
                viewer->removeShape(previous_bbox_id_);
            }

        
            viewer->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, bbox_id);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, bbox_id);

            previous_bbox_id_ = bbox_id;
        }

        template <typename PointCloudType> 
        // Using template, could have either normal or XYZ cloud
        void streamPointCloud(const typename pcl::PointCloud<PointCloudType>::Ptr& cloud) {
      
            std::string cloud_id = "cloud_" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

            //Remove previous pointcloud 
            if(!previous_cloud_id_.empty()) {
                viewer->removePointCloud(previous_cloud_id_);
            }

            if(std::is_same<PointCloudType, pcl::PointXYZRGB>::value || std::is_same<PointCloudType, pcl::PointNormal>::value) {
                viewer->addPointCloud<PointCloudType>(cloud, cloud_id);
            } else {
                std::string error_msg = "streamPointCloud() error: unsupported point cloud type";
                RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
            }

            previous_cloud_id_ = cloud_id;
            //createBoundingBox(cloud);
        }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_sub_;
    std::string previous_cloud_id_;
    std::string previous_bbox_id_;
    std::string filterType_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointCloudPipeline>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}