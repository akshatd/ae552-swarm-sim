#ifndef WORLD_H
#define WORLD_H

#include <atomic>
#include <barrier>
#include <functional>
#include <map>
#include <stop_token>
#include <string>
#include <vector>

#include "drone.h"

constexpr std::chrono::milliseconds kDefaultSleepTime(100);
constexpr float                     kAirDensity = 1.225;                        // at sea level, kg/m^3
constexpr float                     kCd         = 0.47;                         // for a sphere
constexpr float                     kA          = 0.1;                          // cross sectional area, m^2
constexpr float                     kDrag       = 0.5 * kCd * kA * kAirDensity; // this is all of 1/2 * Cd * A * rho
class World {
	public:
		World(std::vector<Drone> drones, Point3d size);
		~World();
		void Start(std::function<void(std::map<std::string, Attitude>)> callback);
		void Stop();

	private:
		std::vector<Drone>              drones_;
		Point3d                         size_;
		std::atomic_bool                running_;
		std::map<std::string, Point3d>  drone_control_outs_;
		std::map<std::string, Attitude> drone_attitudes_;

		void     runDrone(std::stop_token stop_token, Drone &drone, std::barrier<std::function<void()>> &sync);
		Attitude calculateDroneAttitude(Drone &drone, Point3d control_out);
		Attitude applyDrag(Attitude attitude);
		Attitude checkBounds(Attitude attitude, Point3d size);
};

#endif