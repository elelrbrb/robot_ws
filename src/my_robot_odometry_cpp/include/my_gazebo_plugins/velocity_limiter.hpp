// ════════════════════════════════════════════════════
// 커스텀 Gazebo 플러그인: 속도 제한기
    // 기능: diff_drive에 전달되기 전에 cmd_vel의 속도를 클램핑
        // → 안전 장치 역할 (Day 43에서 ROS 노드로도 구현하지만,
        // 플러그인에서 하면 1000Hz로 동작 → 더 안전!)
// ════════════════════════════════════════════════════

#ifndef MY_GAZEBO_PLUGINS__VELOCITY_LIMITER_HPP_
#define MY_GAZEBO_PLUGINS__VELOCITY_LIMITER_HPP_

#include <gazebo/gazebo.hh>
    // Gazebo 핵심 헤더
#include <gazebo/physics/physics.hh>
    // ModelPtr, JointPtr 등
#include <gazebo/common/common.hh>
    // Time, Event 등

#include <gazebo_ros/node.hpp>
    // gazebo_ros::Node → ROS 2와 통신

#include <geometry_msgs/msg/twist.hpp>

#include <memory>

namespace my_gazebo_plugins
{

    
class VelocityLimiter : public gazebo::ModelPlugin
    // gazebo::ModelPlugin 상속!
    // → Load(), OnUpdate() 등을 오버라이드
{
public:
    VelocityLimiter() = default;
    ~VelocityLimiter() override = default;

    /// 플러그인 로드 시 1회 호출
    void Load(
        gazebo::physics::ModelPtr model,
        sdf::ElementPtr sdf
    ) override;

private:
    /// 매 물리 스텝마다 호출
    void OnUpdate(const gazebo::common::UpdateInfo & info);

    /// cmd_vel 수신 콜백
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

    // ROS 2 노드
    gazebo_ros::Node::SharedPtr ros_node_;

    // 구독/발행
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_limited_pub_;

    // 이벤트 연결
    gazebo::event::ConnectionPtr update_connection_;

    // 파라미터
    double max_linear_{0.5};     // 최대 선속도 [m/s]
    double max_angular_{1.0};    // 최대 각속도 [rad/s]
    double max_linear_accel_{0.5};  // 최대 선가속도 [m/s²]

    // 현재 상태
    double current_linear_{0.0};
    double current_angular_{0.0};
    double target_linear_{0.0};
    double target_angular_{0.0};
};

// ★ 필수! Gazebo에 이 클래스를 플러그인으로 등록
GZ_REGISTER_MODEL_PLUGIN(VelocityLimiter)
    // 이 매크로가 없으면 Gazebo가 플러그인을 찾을 수 없다!
    // 내부적으로: extern "C" 함수를 생성하여 동적 로딩(dlopen) 가능하게 함

}  // namespace my_gazebo_plugins

#endif