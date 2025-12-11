#include "RapidTrajectoryGenerator.h"

#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"

using RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator;

namespace
{
std::string toString(RapidTrajectoryGenerator::InputFeasibilityResult result)
{
  switch (result)
  {
    case RapidTrajectoryGenerator::InputFeasible:
      return "feasible";
    case RapidTrajectoryGenerator::InputIndeterminable:
      return "indeterminable";
    case RapidTrajectoryGenerator::InputInfeasibleThrustHigh:
      return "infeasible: thrust too high";
    case RapidTrajectoryGenerator::InputInfeasibleThrustLow:
      return "infeasible: thrust too low";
    case RapidTrajectoryGenerator::InputInfeasibleRates:
      return "infeasible: body rates too high";
    default:
      return "unknown";
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("rapid_trajectory_test_node");

  const ::Vec3 x0(0.0, 0.0, 0.0);
  const ::Vec3 v0(0.0, 0.0, 0.0);
  const ::Vec3 a0(0.0, 0.0, 0.0);
  const ::Vec3 gravity(0.0, 0.0, -9.81);

  RapidTrajectoryGenerator trajectory(x0, v0, a0, gravity);
  trajectory.SetGoalPosition(::Vec3(1.0, 0.5, 1.0));
  trajectory.SetGoalVelocity(::Vec3(0.0, 0.0, 0.0));
  trajectory.SetGoalAcceleration(::Vec3(0.0, 0.0, 0.0));

  constexpr double tf = 2.0;
  trajectory.Generate(tf);

  const auto feasibility = trajectory.CheckInputFeasibility(0.5, 15.0, 10.0, 0.05);
  RCLCPP_INFO(logger, "Input feasibility: %s", toString(feasibility).c_str());

  const std::array<double, 5U> samples{{0.0, tf * 0.25, tf * 0.5, tf * 0.75, tf}};
  for (double t : samples)
  {
    const auto position = trajectory.GetPosition(t);
    const auto velocity = trajectory.GetVelocity(t);
    const auto acceleration = trajectory.GetAcceleration(t);

    RCLCPP_INFO(
      logger,
      "t=%.2f pos=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f) acc=(%.3f, %.3f, %.3f)",
      t,
      position.x,
      position.y,
      position.z,
      velocity.x,
      velocity.y,
      velocity.z,
      acceleration.x,
      acceleration.y,
      acceleration.z);
  }

  rclcpp::shutdown();
  return 0;
}
