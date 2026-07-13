#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <atomic>
#include <map>
#include <string>
#include "ocs2_ros_interfaces/command/IMarkerControl.h"

namespace ocs2 {

    /**
     * Joystick mapping configuration structure
     */
    struct JoystickMapping {
        // Axes mapping
        int left_stick_x = 0;      // Left stick horizontal
        int left_stick_y = 1;      // Left stick vertical
        int right_stick_x = 3;     // Right stick horizontal
        int right_stick_y = 4;     // Right stick vertical
        int left_trigger = 2;      // Left trigger
        int right_trigger = 5;     // Right trigger
        int dpad_x = 6;            // D-pad horizontal
        int dpad_y = 7;            // D-pad vertical

        // Buttons mapping
        int a_button = 0;          // A button
        int b_button = 1;          // B button
        int x_button = 2;          // X button
        int y_button = 3;          // Y button
        int lb_button = 4;         // Left bumper
        int rb_button = 5;         // Right bumper
        int back_button = 6;       // Back button
        int start_button = 7;      // Start button
        int left_stick_press = 9;  // Left stick press
        int right_stick_press = 10; // Right stick press
    };

    /**
     * Joystick marker wrapper that handles joystick input and controls marker operations.
     * This class acts as a wrapper/adapter between joystick input and marker control.
     * It is decoupled from specific marker implementations through the IMarkerControl interface.
     */
    class JoystickMarkerWrapper {
    public:
        /**
         * Constructor
         * @param node ROS node handle
         * @param markerControl Pointer to marker control interface
         * @param linearScale Scale factor for linear movement
         * @param angularScale Scale factor for angular movement
         * @param updateRate Update rate for joystick processing (Hz)
         * @param mapping Custom joystick mapping configuration
         */
        JoystickMarkerWrapper(
            rclcpp::Node::SharedPtr node,
            IMarkerControl* markerControl,
            double linearScale = 0.005,
            double angularScale = 0.05,
            double updateRate = 20.0,
            const JoystickMapping& mapping = JoystickMapping{});

        /**
         * Destructor
         */
        ~JoystickMarkerWrapper() = default;

        /**
         * Enable joystick control
         */
        void enable();

        /**
         * Disable joystick control
         */
        void disable();

        /**
         * Check if joystick control is enabled
         * @return true if enabled, false otherwise
         */
        bool isEnabled() const { return enabled_.load(); }

        /**
         * Set linear movement scale
         * @param scale Scale factor
         */
        void setLinearScale(double scale) { linearScale_ = scale; }

        /**
         * Set angular movement scale
         * @param scale Scale factor
         */
        void setAngularScale(double scale) { angularScale_ = scale; }

        /**
         * Set update rate
         * @param rate Update rate in Hz
         */
        void setUpdateRate(double rate) { updateRate_ = rate; }

        /**
         * Set joystick mapping configuration
         * @param mapping New mapping configuration
         */
        void setJoystickMapping(const JoystickMapping& mapping) { mapping_ = mapping; }

        /**
         * Get current joystick mapping configuration
         * @return Current mapping configuration
         */
        const JoystickMapping& getJoystickMapping() const { return mapping_; }

        /**
         * Sync joystick position with external position update
         * This method is safe to call even when joystick control is disabled
         * @param position New position
         * @param orientation New orientation
         */
        void syncExternalPosition(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

    private:
        /**
         * Joystick callback function
         * @param msg Joy message
         */
        void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

        /**
         * Process button inputs
         * @param msg Joy message
         */
        void processButtons(const sensor_msgs::msg::Joy::SharedPtr msg);

        /**
         * Process axis inputs for position and orientation control
         * @param msg Joy message
         */
        void processAxes(const sensor_msgs::msg::Joy::SharedPtr msg);

        /**
         * Update marker position based on joystick input
         * @param position New position
         * @param orientation New orientation
         */
        void updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

        /**
         * Apply deadzone to input value
         * @param value Input value
         * @param deadzone Deadzone threshold
         * @return Filtered value
         */
        double applyDeadzone(double value, double deadzone = 0.1) const;

        /**
         * Sync current pose with marker position
         * Updates currentPosition_ and currentOrientation_ from marker control
         */
        void syncCurrentPoseWithMarker();

        // ROS components
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystickSubscriber_;

        // Marker control interface
        IMarkerControl* markerControl_;

        // Control parameters
        double linearScale_;
        double angularScale_;
        double updateRate_;

        // Joystick mapping configuration
        JoystickMapping mapping_;

        // State management
        std::atomic<bool> enabled_;
        std::mutex stateMutex_;

        // Timing control
        rclcpp::Time lastUpdateTime_;
        rclcpp::Time lastButtonTime_;
        double buttonCooldownDuration_;

        // Button state tracking
        bool anyButtonPressed_;
        std::map<int, bool> lastButtonStates_; // Track last state of buttons using mapping

        // Current joystick position and orientation
        Eigen::Vector3d currentPosition_;
        Eigen::Quaterniond currentOrientation_;
    };

} // namespace ocs2 