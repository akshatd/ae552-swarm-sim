#ifndef DRONE_H
#define DRONE_H

#include <iostream>
#include <string>

struct Point3d {
		float x;
		float y;
		float z;

		friend std::ostream& operator<<(std::ostream& os, const Point3d& point);
};

struct Attitude {
		Point3d x;
		Point3d dx;

		friend std::ostream& operator<<(std::ostream& os, const Attitude& Attitude);
};

class Drone {
	public:
		Drone(std::string name, Attitude attitude);
		~Drone();
		friend std::ostream& operator<<(std::ostream& os, const Drone& drone);

	private:
		std::string name_;
		Attitude    attitude_;
};

#endif
