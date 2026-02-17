// ════════════════════════════════════════════════════════════
// Day 40 — ROS 2 오도메트리 노드 (C++ 구현)
// Day 39 Python 노드의 C++ 포팅.
    // 차이점:
        // 1. Smart Pointer로 메모리 관리
        // 2. 타입 안전성 (컴파일 타임 에러)
        // 3. 10~100배 빠른 실행 속도
// ════════════════════════════════════════════════════════════

#include "my_robot_odometry_cpp/odometry_node.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"
  // 파라미터 콜백의 반환 타입을 정의한 메시지 헤더를 포함

#include <cmath>
#include <chrono>
  // std::chrono → 나노초 정밀도 시간 관리

namespace my_robot_odometry_cpp
{


// ════════════════════════════════════════
// 생성자
// ════════════════════════════════════════
OdometryNode::OdometryNode(const rclcpp::NodeOptions & options)
: Node("odometry_node_cpp", options)
    // 부모 클래스(rclcpp::Node) 생성자 호출
    // 노드 이름: "odometry_node_cpp"
, prev_time_(this->get_clock()->now())
    // 시간 초기화: 현재 시뮬 시간 (use_sim_time=true일 때 Gazebo 시간)
{

  // ──── 파라미터 선언 + 읽기 ────
  this->declare_parameter("wheel_radius", 0.0325);
  this->declare_parameter("wheel_separation", 0.345);
  this->declare_parameter("ticks_per_rev", 2244);
  this->declare_parameter("odom_frame", std::string("odom"));
  this->declare_parameter("base_frame", std::string("base_footprint"));
  this->declare_parameter("publish_tf", false);
  this->declare_parameter("left_joint_name", std::string("left_wheel_joint"));
  this->declare_parameter("right_joint_name", std::string("right_wheel_joint"));

  const double wr = this->get_parameter("wheel_radius").as_double();
  const double ws = this->get_parameter("wheel_separation").as_double();
  const int tpr = this->get_parameter("ticks_per_rev").as_int();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  left_joint_name_ = this->get_parameter("left_joint_name").as_string();
  right_joint_name_ = this->get_parameter("right_joint_name").as_string();


  // ──── 오도메트리 계산기 생성 ────
  odom_calc_ = std::make_unique<DifferentialDriveOdometry>(wr, ws, tpr);
    // std::make_unique<T>(args...):
        // → new DifferentialDriveOdometry(wr, ws, tpr)를 안전하게!
        // → 예외가 발생해도 메모리 누수 없음!
        // → C++14부터 권장, raw new는 사용 금지!



// ──── 파라미터 변경 콜백 등록 ────
  // add_on_set_parameters_callback → ROS 2 Humble에서 파라미터 변경을 감지하는 API
  // SetParametersResult를 반환해야 ROS 2가 “성공적으로 변경됐다”라고 인식
param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    -> rcl_interfaces::msg::SetParametersResult
    {
        // 파라미터 변경을 허용
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : params) {
            if (param.get_name() == "wheel_separation") {
                double new_ws = param.as_double();
                odom_calc_->set_wheel_separation(new_ws);
                RCLCPP_INFO(this->get_logger(),
                    "wheel_separation updated to %.3f", new_ws);
            }
        }
        return result;
    });



  // ──── 구독자 생성 ────
  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
      // QoS 큐 크기: 최대 10개 메시지 버퍼링
      // 처리가 밀리면 오래된 메시지부터 버림
    std::bind(&OdometryNode::joint_state_callback, this, std::placeholders::_1)
      // std::bind: 멤버 함수를 콜백으로 등록
      // &OdometryNode::joint_state_callback: 함수 포인터
      // this: 이 객체의 멤버 함수임을 지정
      // std::placeholders::_1: 첫 번째 인자(msg)를 전달
    
      // Python에서는 self.callback 한 줄이면 되지만, C++에서는 멤버 함수 포인터 + 객체 바인딩이 필요!
      // 대안: 람다 (C++14+)
        // [this](const auto & msg) { joint_state_callback(msg); }
  );


  // ──── 발행자 생성 ────
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_custom", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/odom_path_custom", 10);


  // ──── TF 브로드캐스터 ────
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // *this: 이 노드의 참조를 전달
    // 브로드캐스터가 노드의 clock, name 등을 사용


  // ──── Path 메시지 초기화 ────
  path_msg_.header.frame_id = odom_frame_;

  RCLCPP_INFO(this->get_logger(),
    "[OdometryNode C++] Started. wheel_r=%.4f, wheel_sep=%.3f, tpr=%d",
    wr, ws, tpr
  );
    // RCLCPP_INFO: ROS 2 C++ 로깅 매크로
    // Python의 self.get_logger().info(...)에 대응
    // printf 스타일 포맷팅 (%f, %d)
}



// ════════════════════════════════════════
// 콜백: /joint_states 수신
// ════════════════════════════════════════
void OdometryNode::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg
)
{
  // ---- Step 1: Joint 인덱스 찾기 ----
  int left_idx = -1;
  int right_idx = -1;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    // size_t: 부호 없는 정수 → 배열 인덱스에 적합
    // int를 쓰면 컴파일러 경고! (signed/unsigned 비교)
    if (msg->name[i] == left_joint_name_) {
      left_idx = static_cast<int>(i);
    }
    if (msg->name[i] == right_joint_name_) {
      right_idx = static_cast<int>(i);
    }
  }

  if (left_idx < 0 || right_idx < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Joint names not found! Expected: %s, %s",
      left_joint_name_.c_str(), right_joint_name_.c_str()
    );
      // WARN_THROTTLE: 5000ms(5초)에 1번만 경고 → 로그 폭주 방지
      // .c_str(): std::string → const char* 변환 (printf 스타일용)
    return;
  }


  // ---- Step 2: 현재 값 읽기 ----
  const double current_left = msg->position[left_idx];
  const double current_right = msg->position[right_idx];
  const rclcpp::Time current_time = msg->header.stamp;


  // ---- Step 3: 첫 메시지 처리 ----
  if (!initialized_) {
    prev_left_pos_ = current_left;
    prev_right_pos_ = current_right;
    prev_time_ = current_time;
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "[OdometryNode C++] First message received.");
    return;
  }


  // ---- Step 4: Δ 계산 ----
  const double delta_left = current_left - prev_left_pos_;
  const double delta_right = current_right - prev_right_pos_;
  const double dt = (current_time - prev_time_).seconds();
    // rclcpp::Time 간 뺄셈 → Duration → .seconds() 로 double 변환
    // Python에서 수동으로 sec/nanosec 계산하던 것이 한 줄!

  if (dt <= 0.0) {
    prev_left_pos_ = current_left;
    prev_right_pos_ = current_right;
    prev_time_ = current_time;
    return;
  }


  // ---- Step 5: 오도메트리 갱신 ----
  const auto [x, y, theta] = odom_calc_->update_from_radians(delta_left, delta_right, dt);
    // C++17 Structured Bindings!
        // std::tuple<double, double, double>을 자동으로 x, y, theta에 분배!
        // Python의 x, y, theta = odom_calc.update_from_radians(...)와 동일 느낌!
  const auto [v, omega] = odom_calc_->get_velocity();
  
  



  // ---- Step 6: Odometry 메시지 조립 ----
  auto odom_msg = nav_msgs::msg::Odometry();
    // auto: 타입 추론 → 타입명이 길 때 편리
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  yaw_to_quaternion(theta,
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w
  );
    // 참조 전달로 Quaternion 필드를 직접 채움!
    // → 임시 변수 없이 한 번에!

  // Pose Covariance
  odom_msg.pose.covariance.fill(0.0);
    // std::array<double, 36>::fill(0.0) → 36개 전부 0
  odom_msg.pose.covariance[0]  = 0.01;    // σ²_x
  odom_msg.pose.covariance[7]  = 0.01;    // σ²_y
  odom_msg.pose.covariance[14] = 1e6;     // σ²_z (모름)
  odom_msg.pose.covariance[21] = 1e6;     // σ²_roll
  odom_msg.pose.covariance[28] = 1e6;     // σ²_pitch
  odom_msg.pose.covariance[35] = 0.005;   // σ²_yaw

  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = omega;

  odom_msg.twist.covariance.fill(0.0);
  odom_msg.twist.covariance[0]  = 0.005;
  odom_msg.twist.covariance[7]  = 1e6;
  odom_msg.twist.covariance[14] = 1e6;
  odom_msg.twist.covariance[21] = 1e6;
  odom_msg.twist.covariance[28] = 1e6;
  odom_msg.twist.covariance[35] = 0.003;

  odom_pub_->publish(odom_msg);


  // ---- Step 7: TF 브로드캐스트 ----
  if (publish_tf_) {
    auto tf_msg = geometry_msgs::msg::TransformStamped();
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
      // Odometry에서 이미 계산한 Quaternion을 그대로 복사!
    tf_broadcaster_->sendTransform(tf_msg);
  }


  // ---- Step 8: Path 궤적 ----
  auto pose_stamped = geometry_msgs::msg::PoseStamped();
  pose_stamped.header = odom_msg.header;
  pose_stamped.pose = odom_msg.pose.pose;
  path_msg_.poses.push_back(pose_stamped);
    // push_back: vector의 끝에 추가

  if (path_msg_.poses.size() > max_path_length_) {
    path_msg_.poses.erase(
      path_msg_.poses.begin(),
      path_msg_.poses.begin() + static_cast<long>(path_msg_.poses.size() - max_path_length_)
    );
      // 오래된 포즈 삭제
      // Python의 list slicing보다 복잡하지만, 메모리 효율적
  }

  path_msg_.header.stamp = current_time;
  path_pub_->publish(path_msg_);


  // ---- Step 9: 이전 값 갱신 ----
  prev_left_pos_ = current_left;
  prev_right_pos_ = current_right;
  prev_time_ = current_time;
}


// ════════════════════════════════════════
// Yaw → Quaternion
// ════════════════════════════════════════
void OdometryNode::yaw_to_quaternion(
  double yaw,
  double & qx, double & qy, double & qz, double & qw
)
{
  const double half = yaw / 2.0;
  qx = 0.0;
  qy = 0.0;
  qz = std::sin(half);
  qw = std::cos(half);
}

}  // namespace my_robot_odometry_cpp