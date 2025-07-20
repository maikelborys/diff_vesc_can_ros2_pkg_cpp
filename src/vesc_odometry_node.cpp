/**
 * @file vesc_odometry_node.cpp
 * @brief VESC Odometry Node for ROS2 (C++ Implementation)
 * 
 * Calculates robot odometry from VESC CAN tachometer data for differential drive robot
 * 
 * Robot Configuration:
 * - Wheel diameter: 355.6mm
 * - Wheel separation: 370mm (distance between wheel centers)
 * - Tachometer pulses per wheel revolution: 23
 * - VESC 28 (Left wheel): CAN ID 0x1B1C
 * - VESC 46 (Right wheel): CAN ID 0x1B2E
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>

class VESCOdometryNode : public rclcpp::Node
{
public:
    VESCOdometryNode() : Node("vesc_odometry_node")
    {
        // Declare parameters
        this->declare_parameter("wheel_diameter", 0.3556);
        this->declare_parameter("wheel_separation", 0.370);
        this->declare_parameter("tachometer_pulses_per_rev", 23);
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("publish_rate", 50.0);
        this->declare_parameter("left_vesc_id", 28);
        this->declare_parameter("right_vesc_id", 46);

        // Get parameters
        wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        pulses_per_rev_ = this->get_parameter("tachometer_pulses_per_rev").as_int();
        can_interface_ = this->get_parameter("can_interface").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        left_vesc_id_ = this->get_parameter("left_vesc_id").as_int();
        right_vesc_id_ = this->get_parameter("right_vesc_id").as_int();

        // Calculate CAN message IDs for STATUS_5 messages
        // CAN ID = 0x1B00 + VESC_ID
        vesc_left_status5_id_ = 0x1B00 + left_vesc_id_;
        vesc_right_status5_id_ = 0x1B00 + right_vesc_id_;

        // Calculate wheel circumference and distance per pulse
        wheel_circumference_ = M_PI * wheel_diameter_;
        distance_per_pulse_ = wheel_circumference_ / pulses_per_rev_;

        // Log robot configuration
        RCLCPP_INFO(this->get_logger(), "Robot Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Left VESC ID: %d (CAN ID: 0x%X)", left_vesc_id_, vesc_left_status5_id_);
        RCLCPP_INFO(this->get_logger(), "  Right VESC ID: %d (CAN ID: 0x%X)", right_vesc_id_, vesc_right_status5_id_);
        RCLCPP_INFO(this->get_logger(), "  Wheel diameter: %.3fm", wheel_diameter_);
        RCLCPP_INFO(this->get_logger(), "  Wheel separation: %.3fm", wheel_separation_);
        RCLCPP_INFO(this->get_logger(), "  Wheel circumference: %.3fm", wheel_circumference_);
        RCLCPP_INFO(this->get_logger(), "  Tachometer pulses per revolution: %d", pulses_per_rev_);
        RCLCPP_INFO(this->get_logger(), "  Distance per pulse: %.6fm", distance_per_pulse_);

        // Initialize tachometer tracking
        left_tach_initial_ = std::nullopt;
        right_tach_initial_ = std::nullopt;
        left_tach_current_ = 0;
        right_tach_current_ = 0;
        left_tach_previous_ = 0;
        right_tach_previous_ = 0;

        // Initialize odometry state
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        left_wheel_distance_ = 0.0;
        right_wheel_distance_ = 0.0;
        linear_velocity_ = 0.0;
        angular_velocity_ = 0.0;
        last_time_ = this->get_clock()->now();

        // Create publishers
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        left_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("left_wheel_distance", 10);
        right_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("right_wheel_distance", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("wheel_velocities", 10);

        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create timer for publishing odometry
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&VESCOdometryNode::publishOdometry, this));

        // Initialize CAN interface
        running_ = true;
        if (initializeCAN()) {
            RCLCPP_INFO(this->get_logger(), "✓ Connected to CAN bus: %s", can_interface_.c_str());
            
            // Start CAN message reception thread
            can_thread_ = std::thread(&VESCOdometryNode::canMessageLoop, this);
            RCLCPP_INFO(this->get_logger(), "✓ CAN message reception thread started");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface");
            running_ = false;
        }
    }

    ~VESCOdometryNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down VESC odometry node...");
        running_ = false;

        if (can_socket_ >= 0) {
            close(can_socket_);
        }

        if (can_thread_.joinable()) {
            can_thread_.join();
        }
    }

private:
    // Parameters
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

    bool initializeCAN()
    {
        // Create socket
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error creating CAN socket");
            return false;
        }

        // Get interface index
        struct ifreq ifr;
        strcpy(ifr.ifr_name, can_interface_.c_str());
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error getting interface index for %s", can_interface_.c_str());
            close(can_socket_);
            return false;
        }

        // Bind socket to CAN interface
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error binding CAN socket");
            close(can_socket_);
            return false;
        }

        return true;
    }

    void canMessageLoop()
    {
        RCLCPP_INFO(this->get_logger(), "CAN message loop started...");
        
        struct can_frame frame;
        
        while (running_) {
            fd_set readSet;
            FD_ZERO(&readSet);
            FD_SET(can_socket_, &readSet);
            
            struct timeval timeout;
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;
            
            int result = select(can_socket_ + 1, &readSet, nullptr, nullptr, &timeout);
            
            if (result > 0 && FD_ISSET(can_socket_, &readSet)) {
                ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
                
                if (nbytes == sizeof(struct can_frame)) {
                    processCANMessage(frame);
                }
            } else if (result < 0 && running_) {
                RCLCPP_WARN(this->get_logger(), "CAN receive error");
            }
        }
    }

    void processCANMessage(const struct can_frame& frame)
    {
        if (frame.can_dlc < 6) {
            return;
        }

        try {
            // Parse STATUS_5 message: tachometer (bytes 0-3) + voltage (bytes 4-5)
            int32_t tachometer_raw = (frame.data[0] << 24) | (frame.data[1] << 16) | 
                                    (frame.data[2] << 8) | frame.data[3];
            
            // Convert electrical tachometer to mechanical tachometer
            // VESC reports electrical revolutions (6 electrical = 1 mechanical)
            int32_t tachometer = tachometer_raw / 6;

            std::lock_guard<std::mutex> lock(data_mutex_);
            
            if (frame.can_id == vesc_left_status5_id_) {
                // Left wheel (VESC 28)
                if (!left_tach_initial_.has_value()) {
                    left_tach_initial_ = tachometer;
                    left_tach_previous_ = tachometer;
                    RCLCPP_INFO(this->get_logger(), "Left wheel tachometer initialized: %d", tachometer);
                }
                left_tach_current_ = tachometer;
                
            } else if (frame.can_id == vesc_right_status5_id_) {
                // Right wheel (VESC 46)
                if (!right_tach_initial_.has_value()) {
                    right_tach_initial_ = tachometer;
                    right_tach_previous_ = tachometer;
                    RCLCPP_INFO(this->get_logger(), "Right wheel tachometer initialized: %d", tachometer);
                }
                right_tach_current_ = tachometer;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error parsing CAN message: %s", e.what());
        }
    }

    std::pair<double, double> calculateWheelDistances()
    {
        if (!left_tach_initial_.has_value() || !right_tach_initial_.has_value()) {
            return {0.0, 0.0};
        }

        // Calculate tachometer differences from initial position
        int32_t left_tach_diff = left_tach_current_ - left_tach_initial_.value();
        int32_t right_tach_diff = right_tach_current_ - right_tach_initial_.value();

        // Convert tachometer pulses to distance
        double left_distance = left_tach_diff * distance_per_pulse_;
        double right_distance = right_tach_diff * distance_per_pulse_;

        return {left_distance, right_distance};
    }

    std::pair<double, double> calculateVelocities(double dt)
    {
        if (dt <= 0) {
            return {0.0, 0.0};
        }

        // Calculate tachometer differences since last update
        int32_t left_tach_delta = left_tach_current_ - left_tach_previous_;
        int32_t right_tach_delta = right_tach_current_ - right_tach_previous_;

        // Convert to velocities
        double left_velocity = (left_tach_delta * distance_per_pulse_) / dt;
        double right_velocity = (right_tach_delta * distance_per_pulse_) / dt;

        // Update previous values
        left_tach_previous_ = left_tach_current_;
        right_tach_previous_ = right_tach_current_;

        return {left_velocity, right_velocity};
    }

    void updateOdometry(double left_distance, double right_distance, double dt)
    {
        // Calculate distance traveled by robot center
        double distance_center = (left_distance + right_distance) / 2.0;

        // Calculate change in orientation
        double delta_theta = (right_distance - left_distance) / wheel_separation_;

        // Update robot pose
        double delta_x, delta_y;
        if (std::abs(delta_theta) < 1e-6) {
            // Straight line motion
            delta_x = distance_center * std::cos(theta_);
            delta_y = distance_center * std::sin(theta_);
        } else {
            // Arc motion
            double radius = distance_center / delta_theta;
            delta_x = radius * (std::sin(theta_ + delta_theta) - std::sin(theta_));
            delta_y = radius * (-std::cos(theta_ + delta_theta) + std::cos(theta_));
        }

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // Normalize theta to [-pi, pi]
        while (theta_ > M_PI) {
            theta_ -= 2.0 * M_PI;
        }
        while (theta_ < -M_PI) {
            theta_ += 2.0 * M_PI;
        }

        // Calculate velocities
        if (dt > 0) {
            linear_velocity_ = distance_center / dt;
            angular_velocity_ = delta_theta / dt;
        }
    }

    void publishOdometry()
    {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();

        std::lock_guard<std::mutex> lock(data_mutex_);

        // Calculate current wheel distances
        auto [left_distance, right_distance] = calculateWheelDistances();

        // Calculate velocities
        auto [left_vel, right_vel] = calculateVelocities(dt);

        // Calculate change in distances since last update
        double delta_left = left_distance - left_wheel_distance_;
        double delta_right = right_distance - right_wheel_distance_;

        // Update odometry
        updateOdometry(delta_left, delta_right, dt);

        // Update stored distances
        left_wheel_distance_ = left_distance;
        right_wheel_distance_ = right_distance;

        // Create and publish odometry message
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = current_time;
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_link";

        // Position
        odom_msg->pose.pose.position.x = x_;
        odom_msg->pose.pose.position.y = y_;
        odom_msg->pose.pose.position.z = 0.0;

        // Orientation (quaternion from yaw)
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg->pose.pose.orientation.x = q.x();
        odom_msg->pose.pose.orientation.y = q.y();
        odom_msg->pose.pose.orientation.z = q.z();
        odom_msg->pose.pose.orientation.w = q.w();

        // Velocity
        odom_msg->twist.twist.linear.x = linear_velocity_;
        odom_msg->twist.twist.angular.z = angular_velocity_;

        // Covariance (simplified - should be tuned based on your system)
        odom_msg->pose.covariance[0] = 0.1;   // x
        odom_msg->pose.covariance[7] = 0.1;   // y
        odom_msg->pose.covariance[35] = 0.1;  // yaw
        odom_msg->twist.covariance[0] = 0.1;  // vx
        odom_msg->twist.covariance[35] = 0.1; // vyaw

        odom_publisher_->publish(std::move(odom_msg));

        // Publish wheel distances
        auto left_msg = std::make_unique<std_msgs::msg::Float64>();
        left_msg->data = left_wheel_distance_;
        left_distance_publisher_->publish(std::move(left_msg));

        auto right_msg = std::make_unique<std_msgs::msg::Float64>();
        right_msg->data = right_wheel_distance_;
        right_distance_publisher_->publish(std::move(right_msg));

        // Publish wheel velocities
        auto vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        vel_msg->linear.x = left_vel;   // Left wheel velocity
        vel_msg->linear.y = right_vel;  // Right wheel velocity
        cmd_vel_publisher_->publish(std::move(vel_msg));

        // Publish TF transform
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        tf2::Quaternion tf_q;
        tf_q.setRPY(0, 0, theta_);
        t.transform.rotation.x = tf_q.x();
        t.transform.rotation.y = tf_q.y();
        t.transform.rotation.z = tf_q.z();
        t.transform.rotation.w = tf_q.w();

        tf_broadcaster_->sendTransform(t);

        last_time_ = current_time;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<VESCOdometryNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("vesc_odometry_node"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
