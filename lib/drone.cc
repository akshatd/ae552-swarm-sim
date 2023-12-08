#include "drone.h"

#include <iostream>
std::ostream& operator<<(std::ostream& os, const Point3d& point) {
	os << point.x << ", " << point.y << ", " << point.z;
	return os;
}

std::ostream& operator<<(std::ostream& os, const Attitude& Attitude) {
	os << "x: [" << Attitude.x << "], dx: [" << Attitude.dx << "]";
	return os;
}

Drone::Drone(std::string name, Attitude attitude) {
	name_     = name;
	attitude_ = attitude;
}

Drone::~Drone() {}
std::ostream& operator<<(std::ostream& os, const Drone& drone) {
	os << drone.name_ << ": " << drone.attitude_;
	return os;
}
