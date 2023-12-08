
#include <cxxopts.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "drone.h"

const std::string kDefaultFile = "drones.json";

int main(int argc, char *argv[]) {
	cxxopts::Options options(
		"Drone Swarm Simulator", "A program that simulates a swarm of drones with different algoirthms.");
	options.add_options()( //
		"f,file", "Drone description file (json)", cxxopts::value<std::string>()->default_value(kDefaultFile))(
		"d,debug", "Enable debugging", cxxopts::value<bool>()->default_value("false"))(
		"h,help", "Print usage" //
	);

	auto result = options.parse(argc, argv);

	if (result.count("help")) {
		std::cout << options.help() << '\n';
		return EXIT_SUCCESS;
	}

	bool debug = result["debug"].as<bool>();

	std::string drone_file(kDefaultFile);
	if (result.count("file")) {
		drone_file = result["file"].as<std::string>();
		if (debug) {
			std::cout << "Drone description file: " << drone_file << '\n';
		}
	} else {
		std::cout << "*** WARN: No drone description file provided, using " << kDefaultFile << " ***\n";
	}

	std::vector<Drone> drones;
	if (std::ifstream in_file{drone_file}) {
		nlohmann::json drone_json = nlohmann::json::parse(in_file);
		for (auto &drone : drone_json) {
			drones.emplace_back(
				std::string(drone["name"]),
				Attitude({drone["x"][0], drone["x"][1], drone["x"][2]}, {drone["dx"][0], drone["dx"][1], drone["dx"][2]}));
		}
	} else {
		std::cout << "*** ERROR: " << drone_file << " does not exist\n";
		return EXIT_FAILURE;
	}

	for (Drone &drone : drones) {
		std::cout << drone << '\n';
	}

	return EXIT_SUCCESS;
}
