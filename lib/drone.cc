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

void Drone::setAttitude(Attitude attitude) {
	std::cout << "Updating attitude for " << name_ << " to " << attitude << '\n';
	attitude_ = attitude;
	std::cout << "Updated attitude for " << name_ << " to " << attitude << '\n';
}

Point3d Drone::getControlOut() {
	std::cout << "Getting control output for " << name_ << '\n';
	// do some calculation using the current attitude or something
	Point3d control_out = {0, 0, 0};
	std::cout << "Got control output for " << name_ << ": " << control_out << '\n';
	return control_out;
}
std::string Drone::getName() const { return name_; }

// for iostream
std::ostream& operator<<(std::ostream& os, const Drone& drone) {
	os << drone.name_ << ": " << drone.attitude_;
	return os;
}
