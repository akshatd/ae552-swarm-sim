#ifndef UI_H
#define UI_H

#include <matplot/matplot.h>

#include <map>
#include <string>

#include "drone.h"

class Ui {
	public:
		Ui(Point3d size);
		void Update(std::map<std::string, Attitude> drones);

	private:
		Point3d                size_;
		matplot::figure_handle fig_;
};

#endif