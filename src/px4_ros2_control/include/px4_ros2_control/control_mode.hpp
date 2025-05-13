#pragma once

#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>

class ControlMode : public px4_ros2::ModeBase
{
public:
	explicit ControlMode(rclcpp::Node& node);

	// Called when the mode is activated
	void onActivate() override;
	// Called when the mode is deactivated
	void onDeactivate() override;
	// Serves as the "main loop" and is called at regular intervals.
	void updateSetpoint(float dt_s) override;

private:
	struct ControlCommand {
        float x;       // x position (m) or x velocity (m/s) or roll (-1 to 1)
        float y;       // y position (m) or y velocity (m/s) or pitch (-1 to 1)
        float z;       // z position (m) or z velocity (m/s) or throttle (0 to 1)
        float r;       // yaw angle (rad) or yaw rate (rad/s) or yaw stick (-1 to 1)
    };

	void handleExecute();

	rclcpp::Node& _node;
	rclcpp::Publisher<px4_msgs::msg::VehicleControlMode>::SharedPtr _control_mode_pub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _target_control_x_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _target_control_y_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _target_control_z_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _target_control_r_sub;

	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
	std::shared_ptr<px4_ros2::LandDetected> _land_detected;

	bool _is_landed;
	float _current_yaw;
	ControlCommand _control_command;
};