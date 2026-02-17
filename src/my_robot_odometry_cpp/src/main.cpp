// ════════════════════════════════════════
// C++ 프로그램의 진입점.
    // ROS 2 초기화 → 노드 생성 → spin → 종료
// ════════════════════════════════════════

#include "rclcpp/rclcpp.hpp"
#include "my_robot_odometry_cpp/odometry_node.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
    // ROS 2 초기화: DDS 미들웨어 시작, 시그널 핸들러 등록

  auto node = std::make_shared<my_robot_odometry_cpp::OdometryNode>();
    // shared_ptr로 노드 생성
        // rclcpp::spin이 shared_ptr을 요구하기 때문!
    // unique_ptr이 아닌 이유:
        //   spin 내부에서 노드를 여러 곳에서 참조해야 하므로

  rclcpp::spin(node);
    // 무한 루프: 콜백 대기 + 실행
    // Ctrl+C → SIGINT → 루프 종료

  rclcpp::shutdown();
    // DDS 종료, 자원 해제

  return 0;
}