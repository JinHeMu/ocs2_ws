#include "ocs2_ros_interfaces/command/JoystickMarkerWrapper.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ocs2
{
    JoystickMarkerWrapper::JoystickMarkerWrapper(
        rclcpp::Node::SharedPtr node,
        IMarkerControl* markerControl,
        const double linearScale,
        const double angularScale,
        const double updateRate,
        const JoystickMapping& mapping)
        : node_(std::move(node)),
          markerControl_(markerControl),
          linearScale_(linearScale),
          angularScale_(angularScale),
          updateRate_(updateRate),
          mapping_(mapping),
          enabled_(false),
          lastUpdateTime_(node_->now()),
          lastButtonTime_(node_->now()),
          buttonCooldownDuration_(0.5),
          anyButtonPressed_(false),
          currentPosition_(0.0, 0.0, 1.0),
          currentOrientation_(1.0, 0.0, 0.0, 0.0)
    {
        // Initialize button states using mapping configuration
        lastButtonStates_[mapping_.a_button] = false;
        lastButtonStates_[mapping_.b_button] = false;
        lastButtonStates_[mapping_.x_button] = false;
        lastButtonStates_[mapping_.y_button] = false;
        lastButtonStates_[mapping_.lb_button] = false;
        lastButtonStates_[mapping_.rb_button] = false;
        lastButtonStates_[mapping_.back_button] = false;
        lastButtonStates_[mapping_.start_button] = false;
        lastButtonStates_[mapping_.left_stick_press] = false;
        lastButtonStates_[mapping_.right_stick_press] = false;

        // Create joystick subscriber
        auto joystickCallback = [this](const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            this->joystickCallback(msg);
        };
        joystickSubscriber_ = node_->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, joystickCallback);

        RCLCPP_INFO(node_->get_logger(), "🎮 JoystickMarkerWrapper created");
        RCLCPP_INFO(node_->get_logger(), "🎮 Joystick control is DISABLED by default. Press right stick to enable.");
        RCLCPP_INFO(node_->get_logger(),
                    "🎮 Controls: Right stick=toggle joystick, Left stick=toggle continuous mode, A=switch active arm, B=send position, X/Y=sync position")
        ;
    }

    void JoystickMarkerWrapper::enable()
    {
        enabled_.store(true);
        RCLCPP_INFO(node_->get_logger(), "🎮 Joystick control ENABLED!");
    }

    void JoystickMarkerWrapper::disable()
    {
        enabled_.store(false);
        RCLCPP_INFO(node_->get_logger(), "🎮 Joystick control DISABLED!");
    }

    void JoystickMarkerWrapper::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check update frequency
        auto currentTime = node_->now();
        double timeSinceLastUpdate = (currentTime - lastUpdateTime_).seconds();
        double updateInterval = 1.0 / updateRate_;

        if (timeSinceLastUpdate < updateInterval)
        {
            return;
        }
        lastUpdateTime_ = currentTime;

        // Process buttons first
        processButtons(msg);

        // Process axes if enabled
        if (enabled_.load())
        {
            processAxes(msg);
        }
    }

    void JoystickMarkerWrapper::processButtons(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons.size() <= 10)
        {
            return;
        }

        // Check button states using mapping configuration
        bool yPressed = msg->buttons[mapping_.y_button];
        bool xPressed = msg->buttons[mapping_.x_button];
        bool aPressed = msg->buttons[mapping_.a_button];
        bool bPressed = msg->buttons[mapping_.b_button];
        bool leftStickPressed = msg->buttons[mapping_.left_stick_press];
        bool rightStickPressed = msg->buttons[mapping_.right_stick_press];

        // Detect button press events (rising edge) - only for essential controls
        bool rightStickJustPressed = rightStickPressed && !lastButtonStates_[mapping_.right_stick_press];
        bool leftStickJustPressed = leftStickPressed && !lastButtonStates_[mapping_.left_stick_press];

        // Update last button states
        lastButtonStates_[mapping_.a_button] = aPressed;
        lastButtonStates_[mapping_.b_button] = bPressed;
        lastButtonStates_[mapping_.x_button] = xPressed;
        lastButtonStates_[mapping_.y_button] = yPressed;
        lastButtonStates_[mapping_.left_stick_press] = leftStickPressed;
        lastButtonStates_[mapping_.right_stick_press] = rightStickPressed;

        // Process essential button events (rising edge detection)
        if (rightStickJustPressed)
        {
            // Right stick press: toggle joystick control
            if (enabled_.load())
            {
                disable();
            }
            else
            {
                enable();
                // Automatically sync current pose with marker position
                syncCurrentPoseWithMarker();
                
                // Immediately update marker pose to ensure continuous mode uses the new position
                updateMarkerPose(currentPosition_, currentOrientation_);
                
                RCLCPP_INFO(node_->get_logger(), "🎮 Joystick control enabled - synced with current marker position");
            }
        }

        if (leftStickJustPressed && enabled_.load())
        {
            // Left stick press: toggle continuous mode
            markerControl_->togglePublishMode();
        }

        // Process regular button events (when enabled)
        if (enabled_.load())
        {
             // X and Y buttons: sync current pose with marker
             if (xPressed || yPressed || aPressed || bPressed)
             {
                 syncCurrentPoseWithMarker();
             }
             
            // A button: switch active arm (dual arm mode only)
            if (aPressed && markerControl_->getMode() == IMarkerControl::Mode::DUAL_ARM)
            {
                auto currentArm = markerControl_->getActiveArm();
                auto newArm = currentArm == IMarkerControl::ArmType::LEFT
                                  ? IMarkerControl::ArmType::RIGHT
                                  : IMarkerControl::ArmType::LEFT;
                markerControl_->setActiveArm(newArm);

                // Immediately update marker pose to ensure continuous mode uses the new position
                updateMarkerPose(currentPosition_, currentOrientation_);

                RCLCPP_INFO(node_->get_logger(), "🎮 Switched active arm to: %s",
                            (newArm == IMarkerControl::ArmType::LEFT) ? "LEFT" : "RIGHT");
            }

            // B button: send trajectory
            if (bPressed && !markerControl_->isContinuousMode())
            {
                
                if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
                {
                    markerControl_->sendSingleArmTrajectories();
                    RCLCPP_INFO(node_->get_logger(), "🎮 Sending single arm position via B button.");
                }
                else
                {
                    markerControl_->sendDualArmTrajectories();
                    RCLCPP_INFO(node_->get_logger(), "🎮 Sending dual arm positions via B button.");
                }
            }
        }

        // Update button state tracking
        anyButtonPressed_ = yPressed || xPressed || aPressed || bPressed || leftStickPressed || rightStickPressed;
    }

    void JoystickMarkerWrapper::processAxes(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() <= 7)
        {
            return;
        }

        bool hasValidInput = false;

        // Left stick controls horizontal movement in x-y plane
        double left_stick_x = applyDeadzone(msg->axes[mapping_.left_stick_x]);
        double left_stick_y = applyDeadzone(msg->axes[mapping_.left_stick_y]);

        // Right stick controls rotation (yaw) and vertical movement (z-axis)
        double right_stick_x = applyDeadzone(msg->axes[mapping_.right_stick_x]);
        double right_stick_y = applyDeadzone(msg->axes[mapping_.right_stick_y]);

        // D-pad controls roll and pitch rotation
        double dpad_x = 0.0;
        double dpad_y = 0.0;
        if (msg->axes.size() > mapping_.dpad_x && msg->axes.size() > mapping_.dpad_y)
        {
            dpad_x = applyDeadzone(msg->axes[mapping_.dpad_x], 0.5);
            dpad_y = applyDeadzone(msg->axes[mapping_.dpad_y], 0.5);
        }

        // Check if there's valid position input
        if (std::abs(left_stick_x) > 0.001 || std::abs(left_stick_y) > 0.001 ||
            std::abs(right_stick_y) > 0.001)
        {
            hasValidInput = true;
            // Update position
            currentPosition_.y() += left_stick_x * linearScale_;   // Left stick X: left/right
            currentPosition_.z() += right_stick_y * linearScale_;  // Right stick Y: up/down
            currentPosition_.x() += left_stick_y * linearScale_;   // Left stick Y: forward/backward
        }

        // Check if there's valid rotation input
        double pitch = 0.0;
        double roll = 0.0;
        double yaw = 0.0;

        // Right stick X controls yaw (left/right rotation)
        if (std::abs(right_stick_x) > 0.001)
        {
            hasValidInput = true;
            yaw = right_stick_x * angularScale_;
        }

        // D-pad controls roll and pitch
        if (std::abs(dpad_x) > 0.001)
        {
            hasValidInput = true;
            roll = dpad_x * angularScale_;   // D-pad X: roll (left/right tilt)
        }
        if (std::abs(dpad_y) > 0.001)
        {
            hasValidInput = true;
            pitch = -dpad_y * angularScale_; // D-pad Y: pitch (forward/backward tilt, inverted)
        }

        // Update orientation if there is rotation input
        if (std::abs(pitch) > 0.001 || std::abs(roll) > 0.001 || std::abs(yaw) > 0.001)
        {
            // Create rotation increment
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());

            Eigen::Quaterniond rotationIncrement = yawAngle * pitchAngle * rollAngle;
            currentOrientation_ = currentOrientation_ * rotationIncrement;
            currentOrientation_.normalize();
        }

        // Update marker if there is valid input
        if (hasValidInput)
        {
            updateMarkerPose(currentPosition_, currentOrientation_);
        }
    }

    void JoystickMarkerWrapper::updateMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
    {
        if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
        {
            markerControl_->setSingleArmPose(position, orientation);
            markerControl_->updateMarkerDisplay("Goal", position, orientation);
        }
        else
        {
            // Dual arm mode: update current active arm
            const auto activeArm = markerControl_->getActiveArm();
            markerControl_->setDualArmPose(activeArm, position, orientation);

            const std::string markerName =
                (activeArm == IMarkerControl::ArmType::LEFT) ? "LeftArmGoal" : "RightArmGoal";
            markerControl_->updateMarkerDisplay(markerName, position, orientation);
        }

        // Output debug information
        RCLCPP_DEBUG(node_->get_logger(), "🎮 Updated %s marker position: [%.3f, %.3f, %.3f]",
                     markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM ? "single arm" :
                     markerControl_->getActiveArm() == IMarkerControl::ArmType::LEFT ? "left arm" : "right arm",
                     position.x(), position.y(), position.z());
    }

    double JoystickMarkerWrapper::applyDeadzone(double value, double deadzone) const
    {
        if (std::abs(value) < deadzone)
        {
            return 0.0;
        }
        return value;
    }

    void JoystickMarkerWrapper::syncExternalPosition(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
    {
        // Always update the internal position and orientation, regardless of enabled state
        currentPosition_ = position;
        currentOrientation_ = orientation;
        
        // Log the sync operation
        RCLCPP_DEBUG(node_->get_logger(), "🎮 Synced external position: [%.3f, %.3f, %.3f] (enabled: %s)",
                     position.x(), position.y(), position.z(),
                     enabled_.load() ? "true" : "false");
    }

    void JoystickMarkerWrapper::syncCurrentPoseWithMarker()
    {
        if (!markerControl_)
        {
            RCLCPP_WARN(node_->get_logger(), "🎮 Marker control not available for pose sync");
            return;
        }

        // Get current marker position based on mode
        if (markerControl_->getMode() == IMarkerControl::Mode::SINGLE_ARM)
        {
            auto [pos, orient] = markerControl_->getSingleArmPose();
            currentPosition_ = pos;
            currentOrientation_ = orient;
        }
        else
        {
            auto [pos, orient] = markerControl_->getDualArmPose(markerControl_->getActiveArm());
            currentPosition_ = pos;
            currentOrientation_ = orient;
        }

        RCLCPP_DEBUG(node_->get_logger(), "🎮 Synced current pose with marker: [%.3f, %.3f, %.3f]",
                     currentPosition_.x(), currentPosition_.y(), currentPosition_.z());
    }
} // namespace ocs2
