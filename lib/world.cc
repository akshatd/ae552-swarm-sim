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
	// initialise drone control outputs
	for (Drone &drone : drones_) {
		drone_control_outs_[drone.getName()] = Point3d({0, 0, 0});
	}
	// initialise drone attitudes
	for (Drone &drone : drones_) {
		drone_attitudes_[drone.getName()] = drone.getAttitude();
	}
	running_ = false;
}

World::~World() { Stop(); }

void World::Start(std::function<void(std::map<std::string, Attitude>)> callback) {
	std::cout << "World::Start\n";
	size_t num_drones = drones_.size();
	// +1 for main thread
	std::barrier<std::function<void()>> sync(static_cast<long int>(num_drones + 1), []() noexcept { return; });

	std::vector<std::jthread> drone_threads;
	for (Drone &drone : drones_) {
		std::cout << "World::runDrone for " << drone << '\n';
		drone_threads.emplace_back(
			[this, &drone, &sync](std::stop_token stop_token_t) { runDrone(stop_token_t, drone, sync); });
	}
	running_ = true;
	std::cout << "World::Start waiting for running to end .. \n";
	while (running_) {
		sync.arrive_and_wait();
		sync.arrive_and_wait();
		callback(drone_attitudes_);
	}
}

void World::Stop() {
	std::cout << "stopping\n";
	running_ = false;
}

void World::runDrone(std::stop_token stop_token, Drone &drone, std::barrier<std::function<void()>> &sync) {
	std::string drone_name = drone.getName();
	while (!stop_token.stop_requested()) {
		drone_control_outs_[drone_name] = drone.getControlOut();
		// std::cout << "World::runDrone after drone.getControlOut\n";
		sync.arrive_and_wait();
		drone_attitudes_[drone_name] = calculateDroneAttitude(drone, drone_control_outs_[drone_name]);
		drone.setAttitude(drone_attitudes_[drone_name]);
		// std::cout << "World::runDrone after drone.setAttitude\n";
		sync.arrive_and_wait();
		std::this_thread::sleep_for(kDefaultSleepTime);
	}
}

Attitude World::calculateDroneAttitude(Drone &drone, Point3d control_out) {
	// here we assume the mass is 1
	Attitude attitude = drone_attitudes_[drone.getName()];
	// std::cout << "initial attitude: " << attitude << '\n';
	// calculate attitude for drone
	attitude.dx += control_out;
	std::cout << "after ctrl out attitude: " << attitude << '\n';
	// apply drag
	attitude = applyDrag(attitude);
	// apply physics for gravity?
	// apply collisions with bounds
	attitude.x += attitude.dx;
	// std::cout << "after adding x attitude: " << attitude << '\n';
	attitude = checkBounds(attitude, size_);
	// std::cout << "checking bounds attitude: " << attitude << '\n';
	// apply collisions with other drones
	// calculate final position and return drone attitude
	// std::cout << "World::calculateDroneAttitude for " << drone << ": " << attitude << '\n';
	return attitude;
}

Attitude World::applyDrag(Attitude attitude) {
	// assume mass is 1 and time step is 1 second
	Point3d drag = (attitude.dx * attitude.dx) * kDrag;
	attitude.dx.x > 0 ? attitude.dx.x -= drag.x : attitude.dx.x += drag.x;
	attitude.dx.y > 0 ? attitude.dx.y -= drag.y : attitude.dx.y += drag.y;
	attitude.dx.z > 0 ? attitude.dx.z -= drag.z : attitude.dx.z += drag.z;
	return attitude;
}

// reverse speed(dx) and bring x back in bounds if out of bounds
Attitude World::checkBounds(Attitude attitude, Point3d size) {
	if (attitude.x.x < 0) {
		attitude.dx.x = -attitude.dx.x;
		attitude.x.x  = -attitude.x.x;
	} else if (attitude.x.x > size.x) {
		attitude.dx.x = -attitude.dx.x;
		attitude.x.x  = size.x - (attitude.x.x - size.x);
	}

	if (attitude.x.y < 0) {
		attitude.dx.y = -attitude.dx.y;
		attitude.x.y  = -attitude.x.y;
	} else if (attitude.x.y > size.y) {
		attitude.dx.y = -attitude.dx.y;
		attitude.x.y  = size.y - (attitude.x.y - size.y);
	}

	if (attitude.x.z < 0) {
		attitude.dx.z = -attitude.dx.z;
		attitude.x.z  = -attitude.x.z;
	} else if (attitude.x.z > size.z) {
		attitude.dx.z = -attitude.dx.z;
		attitude.x.z  = size.z - (attitude.x.z - size.z);
	}
	return attitude;
}