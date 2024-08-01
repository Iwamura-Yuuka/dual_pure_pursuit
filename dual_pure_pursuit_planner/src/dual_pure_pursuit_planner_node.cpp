#include "dual_pure_pursuit_planner/dual_pure_pursuit_planner.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dual_pure_pursuit_planner");
  DualPurePursuitPlanner dualpurepursuitplanner;
  dualpurepursuitplanner.process();

  return 0;
}