
#include <atomic>
#include <chrono>
#include <csignal>
#include <cxxopts.hpp>
#include <exception>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

#include "csv.h"
#include "drone.h"
#include "ui.h"
#include "world.h"

const std::string kDefaultFile = "drones.json";

std::atomic_flag shutdown = false;

void signal_handler(int signal) {
	shutdown.test_and_set();
	shutdown.notify_all();
}

int main(int argc, char *argv[]) {
	// std::signal(SIGINT, signal_handler);

	cxxopts::Options options(
		"Drone Swarm Simulator", "A program that simulates a swarm of drones with different algoirthms.");
	options.add_options()( //
		"f,file", "Drone description file (json)", cxxopts::value<std::string>()->default_value(kDefaultFile))(
		"t,time", "Simulation time (seconds)", cxxopts::value<int>()->default_value("30"))(
		"d,debug", "Enable debugging", cxxopts::value<bool>()->default_value("false"))(
		"h,help", "Print usage" //
	);

	auto result = options.parse(argc, argv);

	if (result.count("help")) {
		std::cout << options.help() << '\n';
		return EXIT_SUCCESS;
	}

	bool debug = result["debug"].as<bool>();
	int  time  = result["time"].as<int>();

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
			std::map<std::string, Point3d> target;
			for (auto &t : drone["target"].items()) {
				target[std::string(t.key())] = Point3d({t.value()[0], t.value()[1], t.value()[2]});
			}
			DroneType type;
			if (drone["type"] == "leader") type = CLeader;
			else if (drone["type"] == "follower") type = CFollower;
			else if (drone["type"] == "decentralized") type = Decentralized;
			else {
				std::cout << "*** ERROR: Unknown drone type " << drone["type"] << '\n';
				return EXIT_FAILURE;
			}
			drones.emplace_back(
				std::string(drone["name"]),
				Attitude({drone["x"][0], drone["x"][1], drone["x"][2]}, {drone["dx"][0], drone["dx"][1], drone["dx"][2]}), type,
				target);
		}
	} else {
		std::cout << "*** ERROR: " << drone_file << " does not exist\n";
		return EXIT_FAILURE;
	}

	Point3d size({10, 10, 10});
	try {
		World world(drones, size);
		Ui    ui(size);
		Csv   csv(drone_file + ".csv", drones);
		world.Start([&ui, &csv](std::map<std::string, Attitude> update) {
			ui.Update(update);
			csv.Write(update);
		});
		// shutdown.wait(false);
		std::this_thread::sleep_for(std::chrono::seconds(time));
		std::cout << "Stopping simulation ... \n";
	} catch (std::exception &e) {
		std::cout << "*** ERROR: " << e.what() << '\n';
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
