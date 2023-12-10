#include "drone.h"

#include <iostream>
std::ostream& operator<<(std::ostream& os, const Point3d& point) {
	os << point.x << ", " << point.y << ", " << point.z;
	return os;
}

Point3d Point3d::operator+=(const Point3d& rhs) {
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}

Point3d Point3d::operator-=(const Point3d& rhs) {
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	return *this;
}

Point3d Point3d::operator-() {
	x = -x;
	y = -y;
	z = -z;
	return *this;
}

Point3d Point3d::operator*=(const Point3d& rhs) {
	x *= rhs.x;
	y *= rhs.y;
	z *= rhs.z;
	return *this;
}

Point3d Point3d::operator*=(const float& rhs) {
	x *= rhs;
	y *= rhs;
	z *= rhs;
	return *this;
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

Attitude Drone::getAttitude() { return attitude_; }

void Drone::setAttitude(Attitude attitude) {
	std::cout << "Drone::setAttitude for " << name_ << " to " << attitude << '\n';
	attitude_ = attitude;
}

Point3d Drone::getControlOut() {
	// do some calculation using the current attitude or something
	Point3d control_out = {0, 0, 0};
	// std::cout << "Drone::getControlOut for " << name_ << ": " << control_out << '\n';
	return control_out;
}
std::string Drone::getName() const { return name_; }

// for iostream
std::ostream& operator<<(std::ostream& os, const Drone& drone) {
	os << drone.name_ << ": " << drone.attitude_;
	return os;
}
