#include "csv.h"

#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "drone.h"

Csv::Csv(std::string filename, std::vector<Drone> drones) {
	file_.open(filename);
	drones_ = drones;
	step_   = 0;
	// write header
	file_ << "time_step";
	for (const auto& drone : drones) {
		file_ << ",drone_" + drone.getName();
	}
	file_ << '\n';
}

Csv::~Csv() { file_.close(); }

void Csv::Write(std::map<std::string, Attitude> drone_attitudes_) {
	Point3d dx;
	float   dx_norm;
	file_ << step_++;

	for (const auto& drone : drones_) {
		dx      = drone_attitudes_[drone.getName()].dx;
		dx_norm = std::sqrt(dx.x * dx.x + dx.y * dx.y + dx.z * dx.z);

		file_ << "," << dx_norm;
	}
	file_ << '\n';
}