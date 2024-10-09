#include "cocobot_dwa_planner/cocobot_dwa_planner.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cocobot_dwa_planner");
  CocobotDWA cocobotdwa;
  cocobotdwa.process();

  return 0;
}