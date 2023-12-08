#ifndef WORLD_H
#define WORLD_H

#include <atomic>
#include <barrier>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drone.h"

constexpr uint kDefaultIterLimit = 10;

class World {
	public:
		World(std::vector<Drone> drones, Point3d size);
		~World();
		void Start(uint iter_limit = kDefaultIterLimit);
		void Stop();

	private:
		std::vector<Drone>                                   drones_;
		Point3d                                              size_;
		std::atomic_bool                                     running_;
		std::map<std::string, Point3d>                       drone_control_outs_;
		std::map<std::string, Attitude>                      drone_attitudes_;
		std::unique_ptr<std::barrier<std::function<void()>>> control_calculated_;

		Point3d  getDroneControlOut(Drone &drone);
		Attitude calculateDroneAttitude(Drone &drone);
};

#endif