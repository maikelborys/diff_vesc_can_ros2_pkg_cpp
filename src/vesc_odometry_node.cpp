/**
 * @file vesc_odometry_node.cpp
 * @brief VESC Odometry Node for ROS2 (C++ Implementation)
 * 
 * 🤖 ROBOT ODOMETRY SYSTEM
 * This node calculates pr        RCLCPP_INFO(this->get_logger(), "🤖 ═══════════════════════════════════════════════════════");

        // ═══════════════════════════════════════════════════════════════════
        // 🔄 TACHOMETER STATE INITIALIZATION
        // ═══════════════════════════════════════════════════════════════════
        
        // Initialize tachometer tracking variables
        // std::nullopt indicates that we haven't received the first reading yet
        left_tach_initial_ = std::nullopt;   // Initial left wheel tachometer reading (baseline)
        right_tach_initial_ = std::nullopt;  // Initial right wheel tachometer reading (baseline)
        
        // Current tachometer values (updated in real-time from CAN messages)
        left_tach_current_ = 0;
        right_tach_current_ = 0;
        
        // Previous tachometer values (used for velocity calculations)
        left_tach_previous_ = 0;
        right_tach_previous_ = 0;

        // ═══════════════════════════════════════════════════════════════════
        // 🤖 ROBOT POSE AND ODOMETRY STATE INITIALIZATION
        // ═══════════════════════════════════════════════════════════════════
        
        // Initialize robot pose in world coordinates
        x_ = 0.0;                    // Robot X position (meters, forward/backward)
        y_ = 0.0;                    // Robot Y position (meters, left/right)
        theta_ = 0.0;                // Robot orientation (radians, counterclockwise from X-axis)
        
        // Initialize wheel distance accumulators
        left_wheel_distance_ = 0.0;  // Total distance traveled by left wheel (meters)
        right_wheel_distance_ = 0.0; // Total distance traveled by right wheel (meters)
        
        // Initialize robot velocity state
        linear_velocity_ = 0.0;      // Robot linear velocity (m/s, forward/backward)
        angular_velocity_ = 0.0;     // Robot angular velocity (rad/s, counterclockwise)
        
        // Initialize timing
        last_time_ = this->get_clock()->now();n and velocity by monitoring VESC motor controller
 * tachometer data via CAN bus. It implements differential drive kinematics to determine robot
 * pose (x, y, θ) and publishes odometry information for navigation systems.
 * 
 * 🔧 SYSTEM ARCHITECTURE:
 * ┌─────────────┐    CAN Bus    ┌──────────────┐    ROS2 Topics    ┌─────────────┐
 * │ VESC Motors │ ═══════════> │ Odometry Node│ ═══════════════> │ Navigation  │
 * │ (Tachometer)│              │ (This File)  │                  │ Stack       │
 * └─────────────┘              └──────────────┘                  └─────────────┘
 * 
 * 🛠️ HARDWARE CONFIGURATION:
 * - Robot Type: Differential drive (two independent wheels)
 * - Wheel diameter: 355.6mm (measured wheel circumference ÷ π)
 * - Wheel separation: 370mm (distance between left and right wheel centers)
 * - Motor Type: 23-pole direct drive motors with 3 Hall sensors
 * - Tachometer Resolution: 138 ticks per mechanical wheel revolution (measured)
 * - Left Motor: VESC ID 28 → CAN STATUS_5 ID 0x1B1C
 * - Right Motor: VESC ID 46 → CAN STATUS_5 ID 0x1B2E
 * 
 * 📊 DATA FLOW:
 * 1. VESC controllers send STATUS_5 messages via CAN (50Hz typical)
 * 2. Node extracts tachometer values from CAN frames
 * 3. Converts tachometer ticks to wheel distances
 * 4. Applies differential drive kinematics for robot pose
 * 5. Publishes odometry, TF transforms, and wheel distances
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

/**
 * @class VESCOdometryNode
 * @brief Main odometry calculation node for VESC-based differential drive robot
 * 
 * This class implements a complete odometry system that:
 * 🔄 Receives tachometer data from VESC motor controllers via CAN bus
 * 📐 Converts tachometer ticks to wheel distances using calibrated parameters
 * 🤖 Applies differential drive kinematics to calculate robot pose
 * 📡 Publishes odometry messages and TF transforms for ROS2 navigation
 * 
 * Key Features:
 * - Real-time CAN message processing with dedicated thread
 * - Direct velocity calculation from distance changes
 * - Thread-safe data handling with mutex protection
 * - Comprehensive debug logging for system monitoring
 */
class VESCOdometryNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - Initialize the VESC odometry node
     * 
     * This constructor performs complete system initialization:
     * 1. 📋 Declares and retrieves ROS2 parameters
     * 2. 🔧 Calculates derived parameters (wheel circumference, distance per tick)
     * 3. 🚀 Initializes publishers, TF broadcaster, and timer
     * 4. 🔌 Establishes CAN bus connection
     * 5. 🧵 Starts background CAN message processing thread
     */
    VESCOdometryNode() : Node("vesc_odometry_node")
    {
        // ═══════════════════════════════════════════════════════════════════
        // 📋 PARAMETER DECLARATION AND RETRIEVAL
        // ═══════════════════════════════════════════════════════════════════
        
        // Physical robot parameters
        this->declare_parameter("wheel_diameter", 0.3556);        // Measured wheel diameter (m)
        this->declare_parameter("wheel_separation", 0.370);       // Distance between wheel centers (m)
        this->declare_parameter("tachometer_pulses_per_rev", 23); // Legacy parameter (poles per revolution)
        this->declare_parameter("ticks_per_mechanical_revolution", 138.0); // Real-world calibrated value
        
        // CAN bus configuration
        this->declare_parameter("can_interface", "can0");         // Linux CAN interface name
        
        // System timing
        this->declare_parameter("publish_rate", 10.0);            // Odometry publishing frequency (Hz)
        
        // VESC motor controller IDs
        this->declare_parameter("left_vesc_id", 28);              // Left wheel VESC identifier
        this->declare_parameter("right_vesc_id", 46);             // Right wheel VESC identifier

        // Retrieve all parameters from ROS2 parameter server
        wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        pulses_per_rev_ = this->get_parameter("tachometer_pulses_per_rev").as_int();
        ticks_per_mechanical_revolution_ = this->get_parameter("ticks_per_mechanical_revolution").as_double();
        can_interface_ = this->get_parameter("can_interface").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        left_vesc_id_ = this->get_parameter("left_vesc_id").as_int();
        right_vesc_id_ = this->get_parameter("right_vesc_id").as_int();

        // ═══════════════════════════════════════════════════════════════════
        // 🔧 CALCULATED PARAMETERS AND CAN ID GENERATION
        // ═══════════════════════════════════════════════════════════════════
        
        // Calculate CAN message IDs for VESC STATUS_5 messages
        // VESC Protocol: STATUS_5 CAN ID = 0x1B00 + VESC_ID
        // Example: VESC ID 28 → CAN ID 0x1B00 + 28 = 0x1B1C
        vesc_left_status5_id_ = 0x1B00 + left_vesc_id_;
        vesc_right_status5_id_ = 0x1B00 + right_vesc_id_;

        // Calculate fundamental wheel parameters
        wheel_circumference_ = M_PI * wheel_diameter_;  // Circumference = π × diameter
        
        // 🎯 CRITICAL CALIBRATION PARAMETER
        // This is the most important parameter for accurate odometry!
        // Real-world measurement: 1379 ticks per 10 wheel turns = 137.9 ≈ 138 ticks/turn
        // Motor analysis: 23-pole motor × 3 Hall sensors = 6 electrical states per pole
        // Verification: 138 ÷ 23 = 6 (confirms 23-pole motor configuration)
        distance_per_pulse_raw_ = wheel_circumference_ / ticks_per_mechanical_revolution_;

        // ═══════════════════════════════════════════════════════════════════
        // 📊 SYSTEM CONFIGURATION LOGGING
        // ═══════════════════════════════════════════════════════════════════
        
        // Log complete robot configuration for verification and debugging
        RCLCPP_INFO(this->get_logger(), "🤖 ═══════════════════════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "🤖 VESC ODOMETRY NODE - SYSTEM CONFIGURATION");
        RCLCPP_INFO(this->get_logger(), "🤖 ═══════════════════════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "🔧 MOTOR CONTROLLERS:");
        RCLCPP_INFO(this->get_logger(), "   • Left VESC ID: %d → CAN STATUS_5 ID: 0x%X", left_vesc_id_, vesc_left_status5_id_);
        RCLCPP_INFO(this->get_logger(), "   • Right VESC ID: %d → CAN STATUS_5 ID: 0x%X", right_vesc_id_, vesc_right_status5_id_);
        RCLCPP_INFO(this->get_logger(), "🛞 WHEEL PARAMETERS:");
        RCLCPP_INFO(this->get_logger(), "   • Wheel diameter: %.3f m", wheel_diameter_);
        RCLCPP_INFO(this->get_logger(), "   • Wheel separation: %.3f m", wheel_separation_);
        RCLCPP_INFO(this->get_logger(), "   • Wheel circumference: %.3f m", wheel_circumference_);
        RCLCPP_INFO(this->get_logger(), "📐 TACHOMETER CALIBRATION:");
        RCLCPP_INFO(this->get_logger(), "   • Ticks per mechanical revolution: %.1f (real-world measured)", ticks_per_mechanical_revolution_);
        RCLCPP_INFO(this->get_logger(), "   • Distance per tachometer tick: %.6f m", distance_per_pulse_raw_);
        RCLCPP_INFO(this->get_logger(), "🔌 CAN INTERFACE: %s", can_interface_.c_str());
        RCLCPP_INFO(this->get_logger(), "⏱️  PUBLISH RATE: %.1f Hz", publish_rate_);
        RCLCPP_INFO(this->get_logger(), "🤖 ═══════════════════════════════════════════════════════");

        // ═══════════════════════════════════════════════════════════════════
        // 🔄 TACHOMETER STATE INITIALIZATION
        // ═══════════════════════════════════════════════════════════════════
        
        // Initialize tachometer tracking variables
        // std::nullopt indicates that we haven't received the first reading yet
        left_tach_initial_ = std::nullopt;   // Initial left wheel tachometer reading (baseline)
        right_tach_initial_ = std::nullopt;  // Initial right wheel tachometer reading (baseline)
        
        // Current tachometer values (updated in real-time from CAN messages)
        left_tach_current_ = 0;
        right_tach_current_ = 0;
        
        // Previous tachometer values (used for velocity calculations)
        left_tach_previous_ = 0;
        right_tach_previous_ = 0;

        // ═══════════════════════════════════════════════════════════════════
        // 🤖 ROBOT POSE AND ODOMETRY STATE INITIALIZATION
        // ═══════════════════════════════════════════════════════════════════
        
        // Initialize robot pose in world coordinates
        x_ = 0.0;                    // Robot X position (meters, forward/backward)
        y_ = 0.0;                    // Robot Y position (meters, left/right)
        theta_ = 0.0;                // Robot orientation (radians, counterclockwise from X-axis)
        
        // Initialize wheel distance accumulators
        left_wheel_distance_ = 0.0;  // Total distance traveled by left wheel (meters)
        right_wheel_distance_ = 0.0; // Total distance traveled by right wheel (meters)
        
        // Initialize robot velocity state
        linear_velocity_ = 0.0;      // Robot linear velocity (m/s, forward/backward)
        angular_velocity_ = 0.0;     // Robot angular velocity (rad/s, counterclockwise)
        
        // Initialize timing
        last_time_ = this->get_clock()->now();

        // ═══════════════════════════════════════════════════════════════════
        //  ROS2 PUBLISHERS AND COMMUNICATION SETUP
        // ═══════════════════════════════════════════════════════════════════
        
        // Create ROS2 publishers for different data streams
        // Queue size of 10 provides buffer for message delivery
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_real", 10);
        RCLCPP_INFO(this->get_logger(), "📡 Created publisher: /odom_real (nav_msgs/Odometry)");
        
        left_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("left_wheel_distance", 10);
        RCLCPP_INFO(this->get_logger(), "📡 Created publisher: /left_wheel_distance (std_msgs/Float64)");
        
        right_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("right_wheel_distance", 10);
        RCLCPP_INFO(this->get_logger(), "📡 Created publisher: /right_wheel_distance (std_msgs/Float64)");
        
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_real", 10);
        RCLCPP_INFO(this->get_logger(), "📡 Created publisher: /cmd_vel_real (geometry_msgs/Twist)");

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
    double ticks_per_mechanical_revolution_;
    std::string can_interface_;
    double publish_rate_;
    int left_vesc_id_;
    int right_vesc_id_;

    // CAN IDs
    uint32_t vesc_left_status5_id_;
    uint32_t vesc_right_status5_id_;

    // Calculated values
    double wheel_circumference_;
    double distance_per_pulse_raw_;  // Distance per tachometer tick (mechanical revolutions)

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

    // ********************************************************************************
    // *                             CAN INTERFACE METHODS                           *
    // ********************************************************************************
    
    /**
     * @brief Initialize the CAN socket connection
     * @return true if successful, false otherwise
     * 
     * This method sets up the Linux SocketCAN interface to communicate with VESC controllers.
     * It creates a raw CAN socket and binds it to the specified CAN interface (usually "can0").
     */
    bool initializeCAN()
    {
        // Step 1: Create a raw CAN socket
        // PF_CAN = Protocol Family for CAN bus
        // SOCK_RAW = Raw socket type for direct CAN frame access
        // CAN_RAW = CAN protocol for raw frames
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error creating CAN socket");
            return false;
        }

        // Step 2: Get the network interface index for the CAN interface
        // We need the interface index (not name) to bind the socket
        struct ifreq ifr;
        strcpy(ifr.ifr_name, can_interface_.c_str());  // Copy interface name (e.g., "can0")
        
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error getting interface index for %s", can_interface_.c_str());
            close(can_socket_);
            return false;
        }

        // Step 3: Bind the socket to the CAN interface
        // This connects our socket to the physical CAN bus
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;                    // Address family for CAN
        addr.can_ifindex = ifr.ifr_ifindex;          // Interface index from step 2

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error binding CAN socket to interface %s", can_interface_.c_str());
            close(can_socket_);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "✅ CAN socket successfully bound to interface %s", can_interface_.c_str());
        return true;
    }

    /**
     * @brief Main loop for receiving and processing CAN messages
     * 
     * This method runs in a separate thread and continuously listens for CAN messages.
     * It uses select() with a timeout to avoid blocking indefinitely.
     * When a message is received, it calls processCANMessage() to handle it.
     */
    void canMessageLoop()
    {
        RCLCPP_INFO(this->get_logger(), "🚀 CAN message reception loop started...");
        
        struct can_frame frame;  // Structure to hold received CAN frame
        
        while (running_) {
            // Step 1: Set up file descriptor set for select()
            // This allows us to wait for data with a timeout
            fd_set readSet;
            FD_ZERO(&readSet);                    // Clear the set
            FD_SET(can_socket_, &readSet);        // Add our CAN socket to the set
            
            // Step 2: Set timeout for select() call
            // This prevents the thread from blocking forever if no messages arrive
            struct timeval timeout;
            timeout.tv_sec = 1;      // 1 second timeout
            timeout.tv_usec = 0;     // 0 microseconds
            
            // Step 3: Wait for data to be available on the socket
            // select() returns:
            //   > 0: Number of file descriptors ready for reading
            //   = 0: Timeout occurred
            //   < 0: Error occurred
            int result = select(can_socket_ + 1, &readSet, nullptr, nullptr, &timeout);
            
            if (result > 0 && FD_ISSET(can_socket_, &readSet)) {
                // Step 4: Data is available - read the CAN frame
                ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
                
                if (nbytes == sizeof(struct can_frame)) {
                    // Step 5: Successfully received a complete CAN frame
                    // Extract the actual CAN ID (remove extended frame and other flags)
                    uint32_t actual_id = frame.can_id & CAN_EFF_MASK;
                    
                    // Debug: Log received messages (throttled to avoid spam)
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                         "📨 CAN ID: 0x%X (raw: 0x%X)", actual_id, frame.can_id);
                    
                    // Step 6: Process the received message
                    processCANMessage(frame);
                }
            } else if (result < 0 && running_) {
                // Error occurred during select()
                RCLCPP_WARN(this->get_logger(), "⚠️ CAN receive error during select()");
            }
            // If result == 0, it's just a timeout - continue the loop
        }
        
        RCLCPP_INFO(this->get_logger(), "🛑 CAN message loop stopped");
    }

    /**
     * @brief Process a received CAN message and extract tachometer data
     * @param frame The received CAN frame
     * 
     * This method filters CAN messages to find VESC STATUS_5 messages containing tachometer data.
     * VESC STATUS_5 messages have CAN ID = 0x1B00 + VESC_ID and contain tachometer in bytes 2-3.
     * 
     * CAN Frame Structure for VESC STATUS_5:
     * - Byte 0-1: Other VESC data
     * - Byte 2-3: Tachometer (16-bit signed value)
     * - Byte 4-7: Other VESC data
     */
    void processCANMessage(const struct can_frame& frame)
    {
        // Step 1: Extract the actual CAN ID (remove extended frame and error flags)
        uint32_t actual_id = frame.can_id & CAN_EFF_MASK;
        
        // Step 2: Check if this message is from one of our VESCs
        // We only care about STATUS_5 messages from our left and right wheel VESCs
        bool is_left_vesc = (actual_id == vesc_left_status5_id_);
        bool is_right_vesc = (actual_id == vesc_right_status5_id_);
        
        if (!is_left_vesc && !is_right_vesc) {
            // This message is not from our VESCs - ignore it
            return;
        }
        
        // Step 3: Validate frame size
        // STATUS_5 messages should have at least 6 bytes (we need bytes 2-3 for tachometer)
        if (frame.can_dlc < 6) {
            RCLCPP_WARN(this->get_logger(), "⚠️ CAN frame too short: %d bytes (expected ≥6)", frame.can_dlc);
            return;
        }

        try {
            // Step 4: Extract tachometer data from bytes 2-3
            // Combine two bytes into a 16-bit value (big-endian format)
            int32_t tachometer_raw = (frame.data[2] << 8) | frame.data[3];
            
            // Step 5: Convert from unsigned 16-bit to signed 16-bit
            // If the value is > 32767, it represents a negative number in two's complement
            if (tachometer_raw > 32767) {
                tachometer_raw -= 65536;  // Convert to negative value
            }

            // Step 6: Use the tachometer value directly
            // IMPORTANT: This tachometer reports mechanical revolutions directly
            // Real-world measurement: 138 ticks per mechanical turn (23-pole motor with 3 Hall sensors)
            // No electrical-to-mechanical conversion needed
            int32_t tachometer = tachometer_raw;

            // Step 7: Thread-safe update of tachometer values
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            if (is_left_vesc) {
                updateLeftWheelTachometer(tachometer);
            } else if (is_right_vesc) {
                updateRightWheelTachometer(tachometer);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "❌ Error parsing CAN message: %s", e.what());
        }
    }

    // ********************************************************************************
    // *                        TACHOMETER UPDATE METHODS                            *
    // ********************************************************************************

    /**
     * @brief Update left wheel tachometer value and handle initialization
     * @param tachometer New tachometer reading from left wheel VESC
     */
    void updateLeftWheelTachometer(int32_t tachometer)
    {
        if (!left_tach_initial_.has_value()) {
            // First reading - initialize the baseline
            left_tach_initial_ = tachometer;
            left_tach_previous_ = tachometer;
            RCLCPP_INFO(this->get_logger(), "🟢 Left wheel tachometer initialized: %d", tachometer);
        } else {
            // Check for unexpected large jumps (might indicate data corruption)
            int32_t tach_jump = std::abs(tachometer - left_tach_current_);
            if (tach_jump > 10) { // Threshold for detecting abnormal jumps
                RCLCPP_WARN(this->get_logger(), "⚠️ LEFT WHEEL: Large tachometer jump %d→%d (Δ%d)", 
                           left_tach_current_, tachometer, tach_jump);
            }
            
            // Periodic debug output (every 50 messages to avoid spam)
            static int left_debug_count = 0;
            if (++left_debug_count % 50 == 0) {
                int32_t total_diff = tachometer - left_tach_initial_.value();
                int32_t delta = tachometer - left_tach_current_;
                RCLCPP_INFO(this->get_logger(), "🔄 Left: %d (total: %+d, delta: %+d)", 
                           tachometer, total_diff, delta);
            }
        }
        left_tach_current_ = tachometer;
    }

    /**
     * @brief Update right wheel tachometer value and handle initialization
     * @param tachometer New tachometer reading from right wheel VESC
     */
    void updateRightWheelTachometer(int32_t tachometer)
    {
        if (!right_tach_initial_.has_value()) {
            // First reading - initialize the baseline
            right_tach_initial_ = tachometer;
            right_tach_previous_ = tachometer;
            RCLCPP_INFO(this->get_logger(), "🔵 Right wheel tachometer initialized: %d", tachometer);
        } else {
            // Check for unexpected large jumps (might indicate data corruption)
            int32_t tach_jump = std::abs(tachometer - right_tach_current_);
            if (tach_jump > 10) { // Threshold for detecting abnormal jumps
                RCLCPP_WARN(this->get_logger(), "⚠️ RIGHT WHEEL: Large tachometer jump %d→%d (Δ%d)", 
                           right_tach_current_, tachometer, tach_jump);
            }
            
            // Periodic debug output (every 50 messages to avoid spam)
            static int right_debug_count = 0;
            if (++right_debug_count % 50 == 0) {
                int32_t total_diff = tachometer - right_tach_initial_.value();
                int32_t delta = tachometer - right_tach_current_;
                RCLCPP_INFO(this->get_logger(), "🔄 Right: %d (total: %+d, delta: %+d)", 
                           tachometer, total_diff, delta);
            }
        }
        right_tach_current_ = tachometer;
    }

    // ********************************************************************************
    // *                        ODOMETRY CALCULATION METHODS                         *
    // ********************************************************************************

    /**
     * @brief Calculate wheel distances from tachometer readings
     * @return pair<left_distance, right_distance> in meters
     * 
     * This method converts raw tachometer tick counts to actual wheel distances.
     * It uses the calibrated distance_per_pulse_raw_ value for accurate conversion.
     */
    std::pair<double, double> calculateWheelDistances()
    {
        // Check if tachometers have been initialized
        if (!left_tach_initial_.has_value() || !right_tach_initial_.has_value()) {
            return {0.0, 0.0};  // Return zero distances if not initialized
        }

        // Calculate tachometer differences from initial position (baseline)
        int32_t left_tach_diff = left_tach_current_ - left_tach_initial_.value();
        int32_t right_tach_diff = right_tach_current_ - right_tach_initial_.value();

        // Convert tachometer ticks to actual distances using calibrated conversion
        // Each tick represents distance_per_pulse_raw_ meters of wheel travel
        double left_distance = left_tach_diff * distance_per_pulse_raw_;
        double right_distance = right_tach_diff * distance_per_pulse_raw_;

        return {left_distance, right_distance};
    }

    /**
     * @brief Calculate wheel velocities from distance changes
     * @param left_delta Change in left wheel distance since last update (meters)
     * @param right_delta Change in right wheel distance since last update (meters)
     * @param dt Time interval since last update (seconds)
     * @return pair<left_velocity, right_velocity> in m/s
     * 
     * This method calculates instantaneous wheel velocities using simple physics:
     * velocity = distance_change / time_interval
     */
    std::pair<double, double> calculateWheelVelocities(double left_delta, double right_delta, double dt)
    {
        // Avoid division by zero and ensure reasonable time intervals
        if (dt <= 0.001) {  // Less than 1ms is unrealistic
            return {0.0, 0.0};
        }

        // Calculate velocities using basic physics: v = Δd / Δt
        double left_velocity = left_delta / dt;   // Left wheel velocity (m/s)
        double right_velocity = right_delta / dt; // Right wheel velocity (m/s)

        // Debug: Show velocity calculation details periodically
        static int velocity_debug_count = 0;
        if (++velocity_debug_count % 25 == 0) {  // Every 25 calls (0.5s at 50Hz)
            RCLCPP_INFO(this->get_logger(), 
                       "⚡ VELOCITY: dt=%.3fs | Left: Δ%.4fm → %.3fm/s | Right: Δ%.4fm → %.3fm/s",
                       dt, left_delta, left_velocity, right_delta, right_velocity);
        }

        return {left_velocity, right_velocity};
    }

    // ********************************************************************************
    // *                        ODOMETRY CALCULATION METHODS                         *
    // ********************************************************************************

    /**
     * @brief Update robot odometry using differential drive kinematics
     * @param left_distance Change in left wheel distance since last update (meters)
     * @param right_distance Change in right wheel distance since last update (meters)
     * @param dt Time interval since last update (seconds)
     * @param current_time Current ROS2 timestamp
     * 
     * This method implements standard differential drive kinematics to calculate:
     * - Robot position (x, y) in the world frame
     * - Robot orientation (θ) relative to the initial heading
     * - Robot linear and angular velocities
     */
    void updateOdometry(double left_distance, double right_distance, double dt, rclcpp::Time /*current_time*/)
    {
        // ═══════════════════════════════════════════════════════════════════
        // 📐 DIFFERENTIAL DRIVE KINEMATICS CALCULATIONS
        // ═══════════════════════════════════════════════════════════════════
        
        // Calculate distance traveled by robot center (average of both wheels)
        double distance_center = (left_distance + right_distance) / 2.0;

        // Calculate change in robot orientation using wheel distance difference
        // Positive delta_theta means counterclockwise rotation (right wheel traveled more)
        double delta_theta = (right_distance - left_distance) / wheel_separation_;

        // ═══════════════════════════════════════════════════════════════════
        // 🤖 ROBOT POSE UPDATE (POSITION AND ORIENTATION)
        // ═══════════════════════════════════════════════════════════════════
        
        // Calculate change in robot position using kinematics
        double delta_x, delta_y;
        
        if (std::abs(delta_theta) < 1e-6) {
            // STRAIGHT LINE MOTION: When delta_theta ≈ 0, robot moves in straight line
            // Simple trigonometry: project distance_center in current heading direction
            delta_x = distance_center * std::cos(theta_);
            delta_y = distance_center * std::sin(theta_);
        } else {
            // ARC MOTION: When delta_theta ≠ 0, robot follows curved path
            // Use arc geometry to calculate position change
            double radius = distance_center / delta_theta;  // Instantaneous radius of curvature
            delta_x = radius * (std::sin(theta_ + delta_theta) - std::sin(theta_));
            delta_y = radius * (-std::cos(theta_ + delta_theta) + std::cos(theta_));
        }

        // Update robot position in world coordinates
        x_ += delta_x;      // Update X position (forward/backward)
        y_ += delta_y;      // Update Y position (left/right)
        theta_ += delta_theta; // Update orientation (rotation)

        // Normalize orientation angle to [-π, π] range
        while (theta_ > M_PI) {
            theta_ -= 2.0 * M_PI;   // Remove full rotations (too positive)
        }
        while (theta_ < -M_PI) {
            theta_ += 2.0 * M_PI;   // Remove full rotations (too negative)
        }

        // ═══════════════════════════════════════════════════════════════════
        // ⚡ VELOCITY CALCULATIONS
        // ═══════════════════════════════════════════════════════════════════
        
        // Calculate wheel velocities from distance changes and time interval
        auto [left_wheel_vel, right_wheel_vel] = calculateWheelVelocities(left_distance, right_distance, dt);
        
        // Calculate robot center velocities from wheel velocities
        linear_velocity_ = (left_wheel_vel + right_wheel_vel) / 2.0;        // Linear velocity (m/s)
        angular_velocity_ = (right_wheel_vel - left_wheel_vel) / wheel_separation_; // Angular velocity (rad/s)
        
        // Debug: Show odometry update details periodically
        static int odom_debug_count = 0;
        if (++odom_debug_count % 50 == 0) {  // Every 50 calls (1s at 50Hz)
            RCLCPP_INFO(this->get_logger(), 
                       "🤖 POSE: x=%.3f y=%.3f θ=%.3f° | VEL: %.3fm/s %.1f°/s | ΔL=%.4f ΔR=%.4f dt=%.3fs",
                       x_, y_, theta_ * 180.0 / M_PI, linear_velocity_, 
                       angular_velocity_ * 180.0 / M_PI, left_distance, right_distance, dt);
        }
    }

    // ********************************************************************************
    // *                           PUBLISHING METHODS                                *
    // ********************************************************************************

    void publishOdometry()
    {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();

        std::lock_guard<std::mutex> lock(data_mutex_);

        // Calculate current wheel distances
        auto [left_distance, right_distance] = calculateWheelDistances();

        // Skip velocity calculation if tachometers are not initialized
        if (!left_tach_initial_.has_value() || !right_tach_initial_.has_value()) {
            last_time_ = current_time;
            return;
        }

        // Initialize wheel distances on first run
        static bool first_run = true;
        if (first_run) {
            left_wheel_distance_ = left_distance;
            right_wheel_distance_ = right_distance;
            first_run = false;
            last_time_ = current_time;
            return;
        }

        // Debug: Show wheel distances every 100 cycles
        static int distance_debug_count = 0;
        if (++distance_debug_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Wheel distances: left=%.3f, right=%.3f", 
                       left_distance, right_distance);
        }

        // Calculate change in distances since last update
        double delta_left = left_distance - left_wheel_distance_;
        double delta_right = right_distance - right_wheel_distance_;

        // Debug: Show deltas every 50 cycles
        static int delta_debug_count = 0;
        if (++delta_debug_count % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "Deltas: left=%.6f, right=%.6f (left_dist=%.3f->%.3f, right_dist=%.3f->%.3f)", 
                       delta_left, delta_right, left_wheel_distance_, left_distance, right_wheel_distance_, right_distance);
        }

        // Update odometry using differential drive kinematics
        updateOdometry(delta_left, delta_right, dt, current_time);

        // Debug: Show velocities every 100 cycles
        static int velocity_debug_count = 0;
        if (++velocity_debug_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Velocities: linear=%.3f m/s, angular=%.3f rad/s (delta_left=%.4f, delta_right=%.4f, dt=%.3f)", 
                       linear_velocity_, angular_velocity_, delta_left, delta_right, dt);
        }

        // Update stored distances for next cycle
        left_wheel_distance_ = left_distance;
        right_wheel_distance_ = right_distance;

        // Create and publish odometry message
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = current_time;
        odom_msg->header.frame_id = "odom_real";
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

        // Publish robot velocities (cmd_vel_real)
        auto vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        vel_msg->linear.x = linear_velocity_;    // Robot linear velocity
        vel_msg->angular.z = angular_velocity_;  // Robot angular velocity
        cmd_vel_publisher_->publish(std::move(vel_msg));

        // Publish TF transform
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom_real";
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

// ********************************************************************************
// *                                  MAIN ENTRY POINT                           *
// ********************************************************************************

/**
 * @brief Main function - Initialize and run the VESC odometry node
 * @param argc Command line argument count
 * @param argv Command line arguments
 * @return Exit status
 */
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
