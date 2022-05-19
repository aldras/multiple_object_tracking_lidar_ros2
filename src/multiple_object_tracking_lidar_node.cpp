#include "multiple_object_tracking_lidar.hpp"

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto my_node = std::make_shared<multiple_object_tracking_lidar::MultipleObjectTrackingLidar>(options);
    //auto pcl_object_detection = rclcpp::Node::make_shared("PclObjectDetection");

    exec.add_node(my_node);
    
    exec.spin();
    
    rclcpp::shutdown();
    return 0;
}
