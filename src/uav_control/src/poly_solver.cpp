#include <uav_control/poly_solver.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

constexpr double kEpsilon = 1e-9;

inline double clamp(double value, double lower, double upper)
{
	return std::max(lower, std::min(value, upper));
}

} // namespace

namespace uav_control {

static Eigen::Matrix<double, 6, 6> build_coefficient_matrix(double Tf)
{
	if (Tf <= 0.0) {
		throw std::invalid_argument("Tf must be positive");
	}
	const double t = Tf;
	Eigen::Matrix<double, 6, 6> coef;
	coef <<
		1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 0.5, 0, 0, 0,
		-10.0 / (t * t * t), -6.0 / (t * t), -1.5 / t, 10.0 / (t * t * t), -4.0 / (t * t), 0.5 / t,
		15.0 / (t * t * t * t), 8.0 / (t * t * t), 1.5 / (t * t), -15.0 / (t * t * t * t), 7.0 / (t * t * t), -1.0 / (t * t),
		-6.0 / (t * t * t * t * t), -3.0 / (t * t * t * t), -0.5 / (t * t * t), 6.0 / (t * t * t * t * t), -3.0 / (t * t * t * t), 0.5 / (t * t * t);
	return coef;
}

Poly5Solver::Poly5Solver(double pos0, double vel0, double acc0, double pos1, double vel1, double acc1, double Tf)
{
	Eigen::Matrix<double, 6, 1> state;
	state << pos0, vel0, acc0, pos1, vel1, acc1;
	A_ = build_coefficient_matrix(Tf) * state;
}

double Poly5Solver::get_snap(double t) const
{
	return 24.0 * A_[4] + 120.0 * A_[5] * t;
}

double Poly5Solver::get_jerk(double t) const
{
	return 6.0 * A_[3] + 24.0 * A_[4] * t + 60.0 * A_[5] * t * t;
}

double Poly5Solver::get_acceleration(double t) const
{
	return 2.0 * A_[2] + 6.0 * A_[3] * t + 12.0 * A_[4] * t * t + 20.0 * A_[5] * t * t * t;
}

double Poly5Solver::get_velocity(double t) const
{
	return A_[1] + 2.0 * A_[2] * t + 3.0 * A_[3] * t * t + 4.0 * A_[4] * t * t * t + 5.0 * A_[5] * t * t * t * t;
}

double Poly5Solver::get_position(double t) const
{
	return A_[0] + A_[1] * t + A_[2] * t * t + A_[3] * t * t * t + A_[4] * t * t * t * t + A_[5] * t * t * t * t * t;
}

Polys5Solver::Polys5Solver(double pos0, double vel0, double acc0,
						 const std::vector<double> &pos1,
						 const std::vector<double> &vel1,
						 const std::vector<double> &acc1,
						 double Tf)
{
	const std::size_t N = pos1.size();
	if (vel1.size() != N || acc1.size() != N) {
		throw std::invalid_argument("pos1, vel1, acc1 must have the same length");
	}
	A_.resize(6, static_cast<int>(N));
	Eigen::Matrix<double, 6, Eigen::Dynamic> state(6, static_cast<int>(N));
	state.row(0).setConstant(pos0);
	state.row(1).setConstant(vel0);
	state.row(2).setConstant(acc0);
	state.row(3) = Eigen::Map<const Eigen::VectorXd>(pos1.data(), static_cast<int>(N));
	state.row(4) = Eigen::Map<const Eigen::VectorXd>(vel1.data(), static_cast<int>(N));
	state.row(5) = Eigen::Map<const Eigen::VectorXd>(acc1.data(), static_cast<int>(N));

	A_ = build_coefficient_matrix(Tf) * state;
}

std::vector<double> Polys5Solver::get_position(double t) const
{
	const int cols = static_cast<int>(A_.cols());
	std::vector<double> result(static_cast<std::size_t>(cols));
	const double t2 = t * t;
	const double t3 = t2 * t;
	const double t4 = t3 * t;
	const double t5 = t4 * t;
	Eigen::VectorXd values = A_.row(0) + A_.row(1) * t + A_.row(2) * t2 + A_.row(3) * t3 + A_.row(4) * t4 + A_.row(5) * t5;
	Eigen::Map<Eigen::VectorXd>(result.data(), cols) = values;
	return result;
}

double wrap_to_pi(double angle)
{
	double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
	if (wrapped < 0.0) {
		wrapped += 2.0 * M_PI;
	}
	return wrapped - M_PI;
}

std::pair<double, double> calculate_yaw(const Eigen::Vector3d &vel_dir,
								const Eigen::Vector3d &goal_dir,
								double last_yaw,
								double dt,
								double max_yaw_rate)
{
	if (dt <= 0.0) {
		return {last_yaw, 0.0};
	}

	Eigen::Vector3d vel_n = vel_dir;
	const double vel_norm = vel_n.norm();
	vel_n /= (vel_norm > kEpsilon ? vel_norm : 1.0);

	Eigen::Vector3d goal_n = goal_dir;
	const double goal_dist = goal_n.norm();
	goal_n /= (goal_dist > kEpsilon ? goal_dist : 1.0);

	const double goal_yaw = std::atan2(goal_n.y(), goal_n.x());
	const double delta_yaw = wrap_to_pi(goal_yaw - last_yaw);
	const double weight = 6.0 * std::abs(delta_yaw) / M_PI;

	Eigen::Vector3d dir_des = vel_n + weight * goal_n;
	const double yaw_desired = (goal_dist > 0.5) ? std::atan2(dir_des.y(), dir_des.x()) : last_yaw;

	const double yaw_diff = wrap_to_pi(yaw_desired - last_yaw);
	const double max_yaw_change = max_yaw_rate * M_PI * dt;
	const double yaw_change = clamp(yaw_diff, -max_yaw_change, max_yaw_change);

	const double yaw = wrap_to_pi(last_yaw + yaw_change);
	const double yawdot = yaw_change / dt;
	return {yaw, yawdot};
}

} // namespace uav_control
