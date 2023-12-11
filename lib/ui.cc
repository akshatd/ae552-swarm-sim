#include <ui.h>

#include <iostream>
#include <map>
#include <string>

#include "drone.h"

Ui::Ui(Point3d size) {
	fig_ = matplot::figure(false);
	fig_->add_axes();
	fig_->current_axes()->xlim({0, size.x});
	fig_->current_axes()->ylim({0, size.y});
	fig_->current_axes()->zlim({0, size.z});
	size_ = size;
}

void Ui::Update(std::map<std::string, Attitude> drones) {
	// std::cout << "Ui::Update\n";
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;
	for (const auto &[key, val] : drones) {
		x.push_back(val.x.x);
		y.push_back(val.x.y);
		z.push_back(val.x.z);
	}
	auto p = fig_->current_axes()->scatter3(x, y, z, "filled");
}