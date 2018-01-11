#ifndef MPC_H
#define MPC_H

#include <vector>
#include <limits>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

const size_t N = 20;
const double dt = 0.10;
const double Lf = 2.67;

const size_t x_start      = 0;
const size_t y_start      = x_start + N;
const size_t psi_start    = y_start + N;
const size_t v_start      = psi_start + N;
const size_t cte_start    = v_start + N;
const size_t epsi_start   = cte_start + N;
const size_t delta_start  = epsi_start + N;
const size_t a_start      = delta_start + N - 1;
const double v_ref        = 45.0;

class MPC
{
 public:
  MPC();
  virtual ~MPC();
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> x;
  vector<double> y;
};

#endif /* MPC_H */
