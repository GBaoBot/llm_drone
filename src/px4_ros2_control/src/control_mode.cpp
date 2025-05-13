#include "px4_ros2_control/control_mode.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <px4_ros2/components/node_with_mode.hpp>

// Constants
static const std::string kModeName = "ControlMode";
static const bool kEnableDebugOutput = true;

ControlMode::ControlMode(rclcpp::Node& node): ModeBase(node, kModeName), _node(node)
{
    _node.declare_parameter<std::string>("topics.in.frd_vx", "/fmu/in/FRD/vx");
    _node.declare_parameter<std::string>("topics.in.frd_vy", "/fmu/in/FRD/vy");
    _node.declare_parameter<std::string>("topics.in.frd_vz", "/fmu/in/FRD/vz");
    _node.declare_parameter<std::string>("topics.in.frd_vr", "/fmu/in/FRD/vr");
    _node.declare_parameter<std::string>("topics.in.ned_px", "/fmu/in/NED/px");
    _node.declare_parameter<std::string>("topics.in.ned_py", "/fmu/in/NED/py");
    _node.declare_parameter<std::string>("topics.in.ned_pz", "/fmu/in/NED/pz");
    _node.declare_parameter<std::string>("topics.in.ned_vx", "/fmu/in/NED/vx");
    _node.declare_parameter<std::string>("topics.in.ned_vy", "/fmu/in/NED/vy");
    _node.declare_parameter<std::string>("topics.in.ned_vz", "/fmu/in/NED/vz");
    _node.declare_parameter<std::string>("topics.in.ned_vr", "/fmu/in/NED/vr");

    // Get parameters
    std::string control_frd_vx_topic = _node.get_parameter("topics.in.frd_vx").as_string();
    std::string control_frd_vy_topic = _node.get_parameter("topics.in.frd_vy").as_string();
    std::string control_frd_vz_topic = _node.get_parameter("topics.in.frd_vz").as_string();
    std::string control_frd_vr_topic = _node.get_parameter("topics.in.frd_vr").as_string();
    std::string control_ned_px_topic = _node.get_parameter("topics.in.ned_px").as_string();
    std::string control_ned_py_topic = _node.get_parameter("topics.in.ned_py").as_string();
    std::string control_ned_pz_topic = _node.get_parameter("topics.in.ned_pz").as_string();
    std::string control_ned_vx_topic = _node.get_parameter("topics.in.ned_vx").as_string();
    std::string control_ned_vy_topic = _node.get_parameter("topics.in.ned_vy").as_string();
    std::string control_ned_vz_topic = _node.get_parameter("topics.in.ned_vz").as_string();
    std::string control_ned_vr_topic = _node.get_parameter("topics.in.ned_vr").as_string();
    std::string takeoff_service_name, land_service_name;

    _target_control_x_sub = _node.create_subscription<std_msgs::msg::Float32>(
        control_frd_vx_topic, 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            _control_command.x = msg->data;
        });

    _target_control_y_sub = _node.create_subscription<std_msgs::msg::Float32>(
        control_frd_vy_topic, 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            _control_command.y = msg->data;
        });

    _target_control_z_sub = _node.create_subscription<std_msgs::msg::Float32>(
        control_frd_vz_topic, 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            _control_command.z = msg->data;
        });

    _target_control_r_sub = _node.create_subscription<std_msgs::msg::Float32>(
        control_frd_vr_topic, 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            _control_command.r = msg->data;
        });

    // Initialize vehicle position and trajectory setpoint
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _land_detected = std::make_shared<px4_ros2::LandDetected>(*this);

    // Initialize local variables
    _control_command = {0.0f, 0.0f, 0.0f, 0.0f};
    _current_yaw = 0.0f;
    _is_landed = false;

    RCLCPP_INFO(_node.get_logger(), "ControlMode initialized");
}

void ControlMode::onActivate()
{
    // Called whenever our mode gets selected
    RCLCPP_INFO(_node.get_logger(), "ControlMode activated");
}

void ControlMode::onDeactivate()
{
    // Called when our mode gets deactivated
    RCLCPP_INFO(_node.get_logger(), "ControlMode deactivated");
    _control_command = {0.0f, 0.0f, 0.0f, 0.0f};
}

void ControlMode::updateSetpoint(float dt_s)
{
    // Update the vehicle local position and attitude
    _is_landed = _land_detected->landed();
    _current_yaw = _vehicle_local_position->heading();

    // Execute 
    handleExecute();
}

void ControlMode::handleExecute()
{
    float cos_yaw = std::cos(_current_yaw);
    float sin_yaw = std::sin(_current_yaw);

    float vel_north = _control_command.x * cos_yaw - _control_command.y * sin_yaw;
    float vel_east = _control_command.x * sin_yaw + _control_command.y * cos_yaw;
    float vel_down = _control_command.z;

    Eigen::Vector3f velocity_ned_m_s(vel_north, vel_east, vel_down);
    _trajectory_setpoint->update(velocity_ned_m_s, std::nullopt, std::nullopt, _control_command.r);
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<ControlMode>>(
			     kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}