#include "world.h"

#include <barrier>
#include <chrono>
#include <exception>
#include <fstream>
#include <memory>
#include <thread>
#include <vector>

#include "drone.h"

World::World(std::vector<Drone> drones, Point3d size) {
	drones_ = drones;
	size_   = size;
	// control_calculated(drones.size())
	// initialise drone control outputs
	for (Drone &drone : drones_) {
		drone_control_outs_[drone.getName()] = Point3d({0, 0, 0});
	}
	// initialise drone attitudes
	for (Drone &drone : drones_) {
		drone_attitudes_[drone.getName()] = Attitude({0, 0, 0}, {0, 0, 0});
	}
}

World::~World() { Stop(); }

void World::Start(uint iter_limit) {
	(void)iter_limit;
	size_t                              num_drones = drones_.size();
	std::barrier<std::function<void()>> sync(static_cast<long int>(num_drones), []() noexcept { return; });

	std::vector<std::jthread> drone_threads;
	for (Drone &drone : drones_) {
		drone_threads.emplace_back(
			[this, &drone, &sync](std::stop_token stop_token_t) { runDrone(stop_token_t, drone, sync); });
	}
	while (running_) {
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

void World::Stop() { running_ = false; }

void World::runDrone(std::stop_token stop_token, Drone &drone, std::barrier<std::function<void()>> &sync) {
	std::string drone_name = drone.getName();
	while (!stop_token.stop_requested()) {
		drone_control_outs_[drone_name] = drone.getControlOut();
		// wait for all threads to arrive and wait
		sync.arrive_and_wait();
		drone_attitudes_[drone_name] = calculateDroneAttitude(drone, drone_control_outs_[drone_name]);
		drone.setAttitude(drone_attitudes_[drone_name]);
	}
}

Attitude World::calculateDroneAttitude(Drone &drone, Point3d control_out) {
	// calculate attitude for drone
	// apply physics for friction
	// apply physics for gravity?
	// apply collisions with bounds
	// apply collisions with other drones
	// update drone attitude
	(void)drone;
	(void)control_out;
	return Attitude({0, 0, 0}, {0, 0, 0});
}