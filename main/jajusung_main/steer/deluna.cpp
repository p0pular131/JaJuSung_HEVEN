#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <cmath>

class PointCloudProcessor {
public:
    PointCloudProcessor() {
        sub = nh.subscribe("/livox/lidar", 1, &PointCloudProcessor::callback, this);
        pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("/clustered_centroids", 1);
        pub_midpoints = nh.advertise<visualization_msgs::MarkerArray>("/delaunay_midpoints", 1);
        pub_path = nh.advertise<std_msgs::Float32MultiArray>("/midpoint_gradients", 1);
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // cloud = do_voxel_grid_downsampling(cloud, 0.5);
        cloud = do_passthrough(cloud, "y", y_min_roi, y_max_roi);
        cloud = do_passthrough(cloud, "x", x_min_roi, x_max_roi);
        cloud = do_passthrough(cloud, "z", z_min_roi, z_max_roi);

        if (cloud->points.empty()) {
            ROS_WARN("No points received");
            return;
        }

        std::vector<pcl::PointIndices> cluster_indices = dbscan_clustering(cloud, 1.0, 2);
        std::vector<Eigen::Vector3d> centroids_clusters;

        for (const auto& indices : cluster_indices) {
            Eigen::Vector3d centroid(0, 0, 0);
            for (const auto& idx : indices.indices) {
                centroid += Eigen::Vector3d(cloud->points[idx].x, cloud->points[idx].y, 0.0);
            }
            centroid /= indices.indices.size();
            centroids_clusters.push_back(centroid);
        }

        publish_clusters(centroids_clusters, msg->header);
        if (centroids_clusters.size() > 2) {
            publish_midpoints(centroids_clusters);
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub_clusters, pub_midpoints, pub_path, pub_roi;

    double x_min_roi = 2.5, x_max_roi = 20.0;
    double y_min_roi = -4.0, y_max_roi = 6.0;
    double z_min_roi = -1.2, z_max_roi = 0.5;
    double length_thre = 1.0;
    double angle_constraints = 140.0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr do_voxel_grid_downsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double leaf_size) {
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud(cloud);
        vox.setLeafSize(leaf_size, leaf_size, leaf_size);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        vox.filter(*cloud_filtered);
        return cloud_filtered;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr do_passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& axis, double min, double max) {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName(axis);
        pass.setFilterLimits(min, max);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pass.filter(*cloud_filtered);
        return cloud_filtered;
    }

    std::vector<pcl::PointIndices> dbscan_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double eps, int min_samples) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(eps);
        ec.setMinClusterSize(min_samples);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        return cluster_indices;
    }

    void publish_clusters(const std::vector<Eigen::Vector3d>& centroids_clusters, const std_msgs::Header& header) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (const auto& centroid : centroids_clusters) {
            cloud.push_back(pcl::PointXYZ(centroid.x(), centroid.y(), centroid.z()));
        }
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header = header;
        pub_clusters.publish(output);
        ROS_INFO("Publishing %ld clusters with topic /clustered_centroids", centroids_clusters.size());
    }

    void publish_midpoints(const std::vector<Eigen::Vector3d>& centroids_clusters) {
        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        delete_marker.ns = "avg_points";
        marker_array.markers.push_back(delete_marker);
        pub_midpoints.publish(marker_array);

        std_msgs::Float32MultiArray path;
        std::vector<std::vector<double>> path_candi;

        for (size_t i = 0; i < centroids_clusters.size(); ++i) {
            for (size_t j = i + 1; j < centroids_clusters.size(); ++j) {
                Eigen::Vector3d midpoint = (centroids_clusters[i] + centroids_clusters[j]) / 2.0;
                double distance = midpoint.head<2>().norm();
                if (distance > length_thre) {
                    double gradient = atan2(midpoint.y(), midpoint.x());
                    path_candi.push_back({midpoint.x(), midpoint.y(), gradient});

                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "livox_frame";
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "avg_points";
                    marker.id = marker_array.markers.size();
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.scale.x = 0.20;
                    marker.scale.y = 0.20;
                    marker.scale.z = 0.20;
                    marker.color.r = 1.0;
                    marker.color.a = 1.0;
                    marker.pose.position.x = midpoint.x();
                    marker.pose.position.y = midpoint.y();
                    marker.pose.position.z = 0.0;
                    marker_array.markers.push_back(marker);
                }
            }
        }

        if (!path_candi.empty()) {
            auto min_point = *std::min_element(path_candi.begin(), path_candi.end(), 
                                               [](const std::vector<double>& a, const std::vector<double>& b) {
                                                   return std::hypot(a[0], a[1]) < std::hypot(b[0], b[1]);
                                               });
            for (double val : min_point) {
                path.data.push_back(val);
            }
            pub_path.publish(path);
        }
        pub_midpoints.publish(marker_array);
        ROS_INFO("Published %ld average midpoint markers with topic /delaunay_midpoints", marker_array.markers.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_processor");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}
