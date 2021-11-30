#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H

#include <vector>

struct RobotCommand {
	std::vector<double> position;
	std::vector<double> orientation;
	std::vector<double> velocity;
	std::vector<double> effort;
};

#endif // ROBOT_COMMAND_H
