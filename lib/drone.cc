#include "drone.h"

#include <iostream>
std::ostream& operator<<(std::ostream& os, const Point3d& point) {
	os << point.x << ", " << point.y << ", " << point.z;
	return os;
}

Point3d Point3d::operator+(const Point3d& rhs) { return Point3d(x + rhs.x, y + rhs.y, z + rhs.z); }

Point3d Point3d::operator+=(const Point3d& rhs) {
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}

Point3d Point3d::operator-(const Point3d& rhs) { return Point3d(x - rhs.x, y - rhs.y, z - rhs.z); }

Point3d Point3d::operator-() { return Point3d(-x, -y, -z); }

Point3d Point3d::operator*(const Point3d& rhs) { return Point3d(x * rhs.x, y * rhs.y, z * rhs.z); }

Point3d Point3d::operator*(const float& rhs) { return Point3d(x * rhs, y * rhs, z * rhs); }

Point3d Point3d::operator/(const float& rhs) { return Point3d(x / rhs, y / rhs, z / rhs); }

std::ostream& operator<<(std::ostream& os, const Attitude& Attitude) {
	os << "x: [" << Attitude.x << "], dx: [" << Attitude.dx << "]";
	return os;
}

Drone::Drone(std::string name, Attitude attitude, DroneType type, std::map<std::string, Point3d> target) {
	name_          = name;
	attitude_      = attitude;
	attitude_prev_ = attitude;
	type_          = type;
	target_        = target;
}

Drone::~Drone() {}

Attitude Drone::getAttitude() { return attitude_; }

void Drone::setAttitude(Attitude attitude) {
	// std::cout << "Drone::setAttitude for " << name_ << " to " << attitude << '\n';
	attitude_prev_ = attitude_;
	attitude_      = attitude;
}

Point3d Drone::getControlOut() {
	// basic PID controller to stabilize drone to 0,0,0
	Point3d target(5, 5, 5);
	Point3d kp_diff     = target - attitude_.x;
	Point3d prop        = kp_diff * Kp;
	Point3d kd_diff     = (target - attitude_prev_.x) - (target - attitude_.x);
	Point3d deriv       = kd_diff * Kd;
	Point3d control_out = prop + deriv;
	// std::cout << "Drone::getControlOut for " << name_ << ": " << control_out << '\n';
	return control_out;
}
std::string Drone::getName() const { return name_; }

// for iostream
std::ostream& operator<<(std::ostream& os, const Drone& drone) {
	os << drone.name_ << "(" << (drone.type_ == Leader ? "Leader" : "Follower") << "): " << drone.attitude_
		 << ", Target: [";
	for (const auto& t : drone.target_) {
		os << t.first << " [" << t.second << "], ";
	}
	os << "]";
	return os;
}
