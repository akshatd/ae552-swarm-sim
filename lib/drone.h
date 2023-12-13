#ifndef DRONE_H
#define DRONE_H

#include <iostream>
#include <map>
#include <string>

/*
  This class contains the Drone class and its associated structs.
  This encapsulates the drone's name, attitude and anything else it might do
*/

struct Point3d {
		float x;
		float y;
		float z;

		friend std::ostream& operator<<(std::ostream& os, const Point3d& point);
		Point3d              operator+(const Point3d& rhs);
		Point3d              operator+=(const Point3d& rhs);
		Point3d              operator-(const Point3d& rhs);
		Point3d              operator-();
		Point3d              operator*(const Point3d& rhs);
		Point3d              operator*(const float& rhs);
		Point3d              operator/(const float& rhs);
		Point3d              operator/=(const float& rhs);
};

struct Attitude {
		Point3d x;
		Point3d dx;

		friend std::ostream& operator<<(std::ostream& os, const Attitude& Attitude);
};

constexpr float Kp = 5.0;
constexpr float Ki = 0.0;
constexpr float Kd = -10.0;

enum DroneType { CLeader, CFollower, Decentralized };
constexpr std::string_view kDroneTypeNames[] = {"CLeader", "CFollower", "Decentralized"};
class Drone {
	public:
		Drone(std::string name, Attitude attitude, DroneType type, std::map<std::string, Point3d> target);
		~Drone();
		Attitude    getAttitude();
		void        setAttitude(Attitude attitude, std::map<std::string, Attitude> all_attitudes_prev);
		Point3d     getControlOut();
		std::string getName() const;

		friend std::ostream& operator<<(std::ostream& os, const Drone& drone);

	private:
		std::string                     name_;
		DroneType                       type_;
		std::map<std::string, Point3d>  target_;
		Attitude                        attitude_;
		Attitude                        attitude_prev_;
		std::map<std::string, Attitude> all_attitudes_prev_;
};

#endif
