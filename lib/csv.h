#ifndef CSV_H
#define CSV_H

#include <fstream>
#include <map>
#include <string>

#include "drone.h"

class Csv {
	public:
		Csv(std::string filename, std::vector<Drone> drones);
		~Csv();
		void Write(std::map<std::string, Attitude> drone_attitudes_);

	private:
		std::ofstream      file_;
		std::vector<Drone> drones_;
		size_t             step_;
};

#endif