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

	// initialise drone attitudes
	for (Drone &drone : drones_) {
		drone_attitudes_[drone.getName()]      = drone.getAttitude();
		drone_attitudes_prev_[drone.getName()] = drone.getAttitude();
	}
}

World::~World() {
	world_thread_.request_stop();
	// print final attitudes
	std::cout << "Final attitudes:\n";
	for (Drone &drone : drones_) {
		std::cout << drone.getName() << ": " << drone.getAttitude() << '\n';
	}
}

void World::Start(std::function<void(std::map<std::string, Attitude>)> callback) {
	std::cout << "World::Start\n";
	world_thread_ = std::jthread([this, &callback](std::stop_token stop_token) {
		size_t num_drones = drones_.size();
		// +1 for main thread
		std::barrier<std::function<void()>> sync(static_cast<long int>(num_drones + 1), []() noexcept { return; });

		std::vector<std::jthread> drone_threads;
		for (Drone &drone : drones_) {
			std::cout << "World::runDrone for " << drone << '\n';
			drone_threads.emplace_back(
				[this, &drone, &sync](std::stop_token stop_token_t) { runDrone(stop_token_t, drone, sync); });
		}
		while (!stop_token.stop_requested()) {
			sync.arrive_and_wait();
			sync.arrive_and_wait();
			callback(drone_attitudes_);
		}
	});
	std::cout << "World::Started .. \n";
}

void World::runDrone(std::stop_token stop_token, Drone &drone, std::barrier<std::function<void()>> &sync) {
	std::string drone_name = drone.getName();
	Point3d     control_out;
	while (!stop_token.stop_requested()) {
		drone_attitudes_prev_[drone_name] = drone_attitudes_[drone_name];
		control_out                       = drone.getControlOut();
		// std::cout << "World::runDrone after drone.getControlOut\n";
		sync.arrive_and_wait();
		drone_attitudes_[drone_name] = calculateDroneAttitude(drone, control_out);
		drone.setAttitude(drone_attitudes_[drone_name], drone_attitudes_prev_);
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
	attitude.dx += control_out / kMass * kTimeStep;
	// std::cout << "after ctrl out attitude: " << attitude << '\n';
	// apply gravity
	attitude = applyGravity(attitude);
	// apply drag
	attitude = applyDrag(attitude);
	// apply collisions with bounds
	attitude.x += attitude.dx * kTimeStep;
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
	Point3d drag = (attitude.dx * attitude.dx) * kDrag / kMass * kTimeStep;
	attitude.dx.x > 0 ? attitude.dx.x -= drag.x : attitude.dx.x += drag.x;
	attitude.dx.y > 0 ? attitude.dx.y -= drag.y : attitude.dx.y += drag.y;
	attitude.dx.z > 0 ? attitude.dx.z -= drag.z : attitude.dx.z += drag.z;
	return attitude;
}

Attitude World::applyGravity(Attitude attitude) {
	// assume mass is 1 and time step is 1 second
	attitude.dx.z -= kGravity * kTimeStep;
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