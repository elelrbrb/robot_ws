// src/velocity_limiter.cpp
// ════════════════════════════════════════════════════

#include "my_gazebo_plugins/velocity_limiter.hpp"
#include <algorithm>  // std::clamp (C++17)

namespace my_gazebo_plugins
{

void VelocityLimiter::Load(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr sdf)
{
    // ---- 파라미터 읽기 ----
    if (sdf->HasElement("max_linear")) {
        max_linear_ = sdf->Get<double>("max_linear");
    }
    if (sdf->HasElement("max_angular")) {
        max_angular_ = sdf->Get<double>("max_angular");
    }
    if (sdf->HasElement("max_linear_accel")) {
        max_linear_accel_ = sdf->Get<double>("max_linear_accel");
    }

    // ---- ROS 2 노드 ----
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // 원본 cmd_vel 구독
    cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_raw", 10,
        std::bind(&VelocityLimiter::OnCmdVel, this, std::placeholders::_1)
    );
        // 원본: /cmd_vel_raw → 제한 후: /cmd_vel

    // 제한된 cmd_vel 발행
    cmd_vel_limited_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", 10
    );

    // ---- OnUpdate 연결 ----
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&VelocityLimiter::OnUpdate, this, std::placeholders::_1)
    );

    RCLCPP_INFO(ros_node_->get_logger(),
        "[VelocityLimiter] Loaded. max_v=%.2f, max_w=%.2f",
        max_linear_, max_angular_);
}

void VelocityLimiter::OnCmdVel(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    target_linear_ = msg->linear.x;
    target_angular_ = msg->angular.z;
}

void VelocityLimiter::OnUpdate(
    const gazebo::common::UpdateInfo & info)
{
    // dt 계산 (생략: info.simTime에서 추출)
    double dt = 0.001;  // 1000Hz 가정

    // ---- 속도 클램핑 ----
    double limited_linear = std::clamp(target_linear_, -max_linear_, max_linear_);
        // std::clamp (C++17):
        // clamp(value, min, max) → min ≤ value ≤ max
        // 범위 밖이면 경계값으로!
    double limited_angular = std::clamp(target_angular_, -max_angular_, max_angular_);

    // ---- 가속도 제한 ----
    double max_delta_v = max_linear_accel_ * dt;
    double delta_v = limited_linear - current_linear_;
    if (std::abs(delta_v) > max_delta_v) {
        limited_linear = current_linear_ + std::copysign(max_delta_v, delta_v);
    }
    current_linear_ = limited_linear;
    current_angular_ = limited_angular;

    // ---- 제한된 값 발행 ----
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = limited_linear;
    msg.angular.z = limited_angular;
    cmd_vel_limited_pub_->publish(msg);
}

}  // namespace my_gazebo_plugins