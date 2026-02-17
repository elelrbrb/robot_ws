#ifndef MY_ROBOT_ODOMETRY_CPP__ODOMETRY_NODE_HPP_
#define MY_ROBOT_ODOMETRY_CPP__ODOMETRY_NODE_HPP_

#include <memory>
  // std::shared_ptr, std::unique_ptr → Smart Pointer!
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
  // ROS 2 C++ 핵심: Node, Publisher, Subscription, Timer 등

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "my_robot_odometry_cpp/differential_drive_odometry.hpp"
  // 우리가 만든 오도메트리 계산 클래스


namespace my_robot_odometry_cpp
{


class OdometryNode : public rclcpp::Node
  // rclcpp::Node을 상속!
    // → 이 클래스가 ROS 2 노드로서 동작
    // → create_subscription, create_publisher 등 사용 가능
{
public:
  /// 생성자
    /// explicit: OdometryNode node = "string"; 같은 암묵적 변환 방지
  explicit OdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ──── 콜백 ────
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    // SharedPtr = std::shared_ptr<const JointState>
        // ROS 2는 메시지를 shared_ptr로 전달 → 복사 비용 0!
        // 여러 구독자가 같은 메시지를 공유 가능

  // ──── 유틸리티 ────
  static void yaw_to_quaternion(
    double yaw,
    double & qx, double & qy, double & qz, double & qw
  );
    // 참조(&)로 출력 → 4개 값을 한 번에 반환
        // C++17에서는 structured bindings를 쓸 수도 있지만,
        // ROS 2 메시지의 개별 필드에 직접 대입하는 패턴이라 참조가 더 깔끔


  
  // 파라미터 변경 콜백을 등록하면 ROS 2가 핸들을 반환
    // 이 핸들을 멤버 변수로 저장해 두어야, 콜백이 살아있는 동안 유효하게 유지
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;


  
        
  // ──── 멤버 변수 ────
  // 오도메트리 계산기 (unique_ptr)
  std::unique_ptr<DifferentialDriveOdometry> odom_calc_;
    // unique_ptr: "이 노드만 오도메트리 계산기를 소유"
        // → 소유권이 명확 → 메모리 누수 불가능!
        // → 노드가 파괴되면 자동으로 계산기도 파괴


  // 구독자 (shared_ptr)
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    // SharedPtr인 이유: ROS 2 내부에서 구독자를 관리할 때 공유 필요
        // 우리가 직접 delete할 일이 없음 → 자동 관리


  // 발행자
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;


  // TF 브로드캐스터
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // unique_ptr: 이 노드만 브로드캐스터를 소유

    
  // 이전 상태 (Δ 계산용)
  bool initialized_{false};
    // 첫 메시지를 받았는지 여부
    // Python의 prev_left_pos = None 에 대응
  double prev_left_pos_{0.0};
  double prev_right_pos_{0.0};
  rclcpp::Time prev_time_;


  // Joint 이름
  std::string left_joint_name_;
  std::string right_joint_name_;


  // 파라미터
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;


  // Path 궤적
  nav_msgs::msg::Path path_msg_;
  size_t max_path_length_{5000};
};


} 

#endif