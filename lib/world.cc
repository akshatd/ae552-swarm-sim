#include "world.h"

#include <barrier>
#include <exception>
#include <fstream>
#include <memory>
#include <vector>

#include "drone.h"

World::World(std::vector<Drone> drones, Point3d size)
		: control_calculated_(std::make_unique<std::barrier<std::function<void()>>>(drones.size(), [this]() { return; })) {
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
	// initialize drones
	while (iter_limit-- > 0 && running_) {
		// get control outputs for all drones
		for (Drone &drone : drones_) {
			drone_control_outs_[drone.getName()] = drone.getControlOut();
		}
		// calculate drone attitudes and set them
		for (Drone &drone : drones_) {
			drone_attitudes_[drone.getName()] = calculateDroneAttitude(drone);
			drone.setAttitude(drone_attitudes_[drone.getName()]);
		}
	}
}

void World::Stop() { running_ = false; }

Point3d World::getDroneControlOut(Drone &drone) {
	(void)drone;
	return {0, 0, 0};
}

Attitude World::calculateDroneAttitude(Drone &drone) {
	// calculate attitude for drone
	// apply physics for friction
	// apply physics for gravity?
	// apply collisions with bounds
	// apply collisions with other drones
	// update drone attitude
	(void)drone;
	return Attitude({0, 0, 0}, {0, 0, 0});
}