#include <test_competitor/test_competitor.hpp>

#include <rclcpp/rclcpp.hpp>

#include <ariac_msgs/msg/kitting_task.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto test_competitor = std::make_shared<TestCompetitor>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(test_competitor);
  std::thread([&executor]() { executor.spin(); }).detach();
  
  // // Create a sample kitting task
  // ariac_msgs::msg::KittingTask task;

  // task.agv_number = 4;
  // task.tray_id = 3;
  // task.destination = ariac_msgs::msg::KittingTask::WAREHOUSE;
  
  // ariac_msgs::msg::KittingPart part1;
  // part1.part.color = ariac_msgs::msg::Part::BLUE;
  // part1.part.type = ariac_msgs::msg::Part::BATTERY;
  // part1.quadrant = ariac_msgs::msg::KittingPart::QUADRANT1;

  // task.parts.push_back(part1);

  test_competitor->SendRobotToHome("floor_robot");

  // test_competitor->CompleteKittingTask(task);

  // test_competitor->ChangeFloorRobotTool("kts1", "trays");

  // test_competitor->SendRobotToHome("floor_robot");

  ariac_msgs::msg::Part part_to_pick;
  part_to_pick.type = ariac_msgs::msg::Part::BATTERY;
  part_to_pick.color = ariac_msgs::msg::Part::BLUE;

  test_competitor->PickBinPart(part_to_pick);

  char c;
  std::cin >> c;

  rclcpp::shutdown();
}