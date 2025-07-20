/**
 * @file vesc_odometry_node.hpp
 * @brief Header file for VESC Odometry Node (C++ Implementation)
 */

#ifndef DIFF_VESC_CAN_ROS2_PKG_CPP__VESC_ODOMETRY_NODE_HPP_
#define DIFF_VESC_CAN_ROS2_PKG_CPP__VESC_ODOMETRY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <linux/can.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>

namespace diff_vesc_can_ros2_pkg_cpp
{

class VESCOdometryNode : public rclcpp::Node
{
public:
    VESCOdometryNode();
    ~VESCOdometryNode();

private:
    // Parameter variables
    double wheel_diameter_;
    double wheel_separation_;
    int pulses_per_rev_;
    std::string can_interface_;
    double publish_rate_;
    int left_vesc_id_;
    int right_vesc_id_;

    // CAN IDs
    uint32_t vesc_left_status5_id_;
    uint32_t vesc_right_status5_id_;

    // Calculated values
    double wheel_circumference_;
    double distance_per_pulse_;

    // Tachometer tracking
    std::optional<int32_t> left_tach_initial_;
    std::optional<int32_t> right_tach_initial_;
    int32_t left_tach_current_;
    int32_t right_tach_current_;
    int32_t left_tach_previous_;
    int32_t right_tach_previous_;

    // Odometry state
    double x_;
    double y_;
    double theta_;
    double left_wheel_distance_;
    double right_wheel_distance_;
    double linear_velocity_;
    double angular_velocity_;
    rclcpp::Time last_time_;

    // Thread safety
    std::mutex data_mutex_;

    // Publishers and TF
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_distance_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_distance_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // CAN interface
    int can_socket_;
    std::thread can_thread_;
    std::atomic<bool> running_;

    // Private methods
    bool initializeCAN();
    void canMessageLoop();
    void processCANMessage(const struct can_frame& frame);
    std::pair<double, double> calculateWheelDistances();
    std::pair<double, double> calculateVelocities(double dt);
    void updateOdometry(double left_distance, double right_distance, double dt);
    void publishOdometry();
};

}  // namespace diff_vesc_can_ros2_pkg_cpp

#endif  // DIFF_VESC_CAN_ROS2_PKG_CPP__VESC_ODOMETRY_NODE_HPP_
