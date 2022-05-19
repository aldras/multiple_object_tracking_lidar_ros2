#ifndef MULTIPLE_OBJECT_TRACKING_LIDAR_HPP
#define MULTIPLE_OBJECT_TRACKING_LIDAR_HPP

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/point.hpp>
#include "visualization_msgs/msg/marker.hpp"

#include "opencv2/video/tracking.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>

namespace multiple_object_tracking_lidar
{

// KF init
int stateDim = 4; // [x,y,v_x,v_y]//,w,h]
int measDim = 2;  // [z_x,z_y,z_w,z_h]
int ctrlDim = 0;
cv::KalmanFilter KF0(stateDim, measDim, ctrlDim, CV_32F);
cv::KalmanFilter KF1(stateDim, measDim, ctrlDim, CV_32F);
cv::KalmanFilter KF2(stateDim, measDim, ctrlDim, CV_32F);
cv::KalmanFilter KF3(stateDim, measDim, ctrlDim, CV_32F);
cv::KalmanFilter KF4(stateDim, measDim, ctrlDim, CV_32F);
cv::KalmanFilter KF5(stateDim, measDim, ctrlDim, CV_32F);

/*
cv::Mat state(stateDim, 1, CV_32F);
cv::Mat_<float> measurement(2, 1);
*/

class MultipleObjectTrackingLidar : public rclcpp::Node{
public:
    MultipleObjectTrackingLidar(
        const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
    );
    MultipleObjectTrackingLidar(
        const std::string& name_space,
        const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
    );
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr objID_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster0;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster1;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster3;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster4;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster5;

    std::vector<geometry_msgs::msg::Point> prevClusterCenters;

    std::vector<int> objID; // Output of the data association using KF
                            // measurement.setTo(Scalar(0));

    bool firstFrame = true;
    rclcpp::Clock::SharedPtr clock_;
    std::string frame_id;
    std::string filtered_cloud;

    double euclidean_distance(geometry_msgs::msg::Point &p1, geometry_msgs::msg::Point &p2);
    std::pair<int, int> findIndexOfMin(std::vector<std::vector<float>> distMat);
    void kft(const std_msgs::msg::Float32MultiArray ccs);
    void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);
    void cloud_cb(const sensor_msgs::msg::PointCloud2::ConstPtr &input);
};

}
#endif //MULTIPLE_OBJECT_TRACKING_LIDAR_HPP
