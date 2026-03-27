#pragma once

#include <Eigen/Dense>
#include <utility>
#include <vector>

namespace uav_control {

	class Poly5Solver {
	public:
		Poly5Solver(double pos0, double vel0, double acc0, double pos1, double vel1, double acc1, double Tf);

		double get_snap(double t) const;
		double get_jerk(double t) const;
		double get_acceleration(double t) const;
		double get_velocity(double t) const;
		double get_position(double t) const;

	private:
		Eigen::Matrix<double, 6, 1> A_;
	};

	class Polys5Solver {
	public:
		Polys5Solver(double pos0, double vel0, double acc0,
				const std::vector<double> &pos1,
				const std::vector<double> &vel1,
				const std::vector<double> &acc1,
				double Tf);

		std::vector<double> get_position(double t) const;

	private:
		Eigen::Matrix<double, 6, Eigen::Dynamic> A_;
	};

	double wrap_to_pi(double angle);
	std::pair<double, double> calculate_yaw(const Eigen::Vector3d &vel_dir,
									const Eigen::Vector3d &goal_dir,
									double last_yaw,
									double dt,
									double max_yaw_rate = 0.5);

} // namespace uav_control
