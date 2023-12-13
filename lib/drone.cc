#include "drone.h"

#include <iostream>
#include <map>
#include <vector>

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

Point3d Point3d::operator/=(const float& rhs) {
	x /= rhs;
	y /= rhs;
	z /= rhs;
	return *this;
}

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

void Drone::setAttitude(Attitude attitude, std::map<std::string, Attitude> all_attitudes_prev) {
	// std::cout << "Drone::setAttitude for " << name_ << " to " << attitude << '\n';
	attitude_prev_      = attitude_;
	attitude_           = attitude;
	all_attitudes_prev_ = all_attitudes_prev;
}

Point3d Drone::getControlOut() {
	// basic PID controller to stabilize drone to 0,0,0
	Point3d target(0, 0, 0);
	switch (type_) {
		case CLeader:
			target = target_[name_];
			break;
		case CFollower: {
			// sum of where the leader was + our offset to it
			target = all_attitudes_prev_[target_.begin()->first].x + target_.begin()->second;
		} break;
		case Decentralized: {
			Point3d target_own(0, 0, 0), target_others(0, 0, 0);
			for (const auto& t : target_) {
				if (t.first == name_) {
					target_own = t.second;
				} else {
					target_others += all_attitudes_prev_[t.first].x + t.second;
				}
				target_others /= static_cast<float>(target_.size() - 1);
				target = (target_own + target_others) / 2.0f;
			}
		} break;
		default:
			break;
	}
	// PID to get to target
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
	os << drone.name_ << "(" << kDroneTypeNames[drone.type_] << "): " << drone.attitude_ << ", Target: [";
	for (const auto& t : drone.target_) {
		os << t.first << " [" << t.second << "], ";
	}
	os << "]";
	return os;
}
