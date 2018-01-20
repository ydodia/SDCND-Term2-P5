#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

double v_ref = 125.0; //mph
vector<double> weights;

class FG_eval
{
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars)
  {
    //std::cerr << "vars size is: " << vars.size() << std::endl;
    //std::cerr << "fg size is: " << fg.size() << std::endl;
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0.0;
    // build up the cost
    for(size_t t=0; t<N; ++t)
    {
      fg[0] += weights[0]*CppAD::pow(vars[cte_start + t] , 2);
      fg[0] += weights[1]*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += weights[2]*CppAD::pow(vars[v_start + t] - v_ref*0.44704, 2);
    }
    for(size_t t=0; t<N-1; ++t)
    {
      fg[0] += weights[3]*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += weights[4]*CppAD::pow(vars[a_start + t], 2);
      fg[0] += weights[7]*CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);
    }
    for(size_t t=0; t<N-2; ++t)
    {
      fg[0] += weights[5]*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += weights[6]*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    // initial values
    fg[1 + x_start]     = vars[x_start];
    fg[1 + y_start]     = vars[y_start];
    fg[1 + psi_start]   = vars[psi_start];
    fg[1 + v_start]     = vars[v_start];
    fg[1 + cte_start]   = vars[cte_start];
    fg[1 + epsi_start]  = vars[epsi_start];
    for(size_t t=1; t<N; ++t)
    {
      // state
      AD<double> x0     = vars[x_start      + t - 1];
      AD<double> y0     = vars[y_start      + t - 1];
      AD<double> psi0   = vars[psi_start    + t - 1];
      AD<double> v0     = vars[v_start      + t - 1];
      AD<double> cte0   = vars[cte_start    + t - 1];
      AD<double> epsi0  = vars[epsi_start   + t - 1];
      // actuators
      AD<double> delta0 = vars[delta_start  + t - 1];
      AD<double> a0     = vars[a_start      + t - 1];

      // next time step
      // state
      AD<double> x1     = vars[x_start      + t];
      AD<double> y1     = vars[y_start      + t];
      AD<double> psi1   = vars[psi_start    + t];
      AD<double> v1     = vars[v_start      + t];
      AD<double> cte1   = vars[cte_start    + t];
      AD<double> epsi1  = vars[epsi_start   + t];

      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*CppAD::pow(x0, 2) + coeffs[3]*CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2.0*coeffs[2]*x0 + 3.0*coeffs[3]*CppAD::pow(x0, 2));

      // store the change
      fg[1 + x_start + t]     = x1    - (x0 + v0*CppAD::cos(psi0)*dt);
      fg[1 + y_start + t]     = y1    - (y0 + v0*CppAD::sin(psi0)*dt);
      fg[1 + psi_start + t]   = psi1  - (psi0 + v0*delta0/Lf*dt);
      fg[1 + v_start + t]     = v1    - (v0 + a0*dt);
      fg[1 + cte_start + t]   = cte1  - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t]  = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC()
{
  /*
   * MSE CTE: 0.653984
AVG speed:  74 mph.
Current weights are:
[cte_start + t] = 400
[epsi_start + t] = 10
[v_start + t] - v_ref = 20
[delta_start + t] = 25
[a_start + t] = 1
[delta_start + t + 1] - [delta_start + t] = 500
[a_start + t + 1] - [a_start + t] = 0.5
[delta_start + t] * [v_start + t] = 6000
steps = 300
v_ref = 125
   */

  x = std::vector<double>(N, 0);
  y = std::vector<double>(N, 0);
  weights = std::vector<double>(n_weights, 0);
  weights[0] = 400;//100; // [cte_start + t]
  weights[1] = 10;//4500; // [epsi_start + t]
  weights[2] = 20; // [v_start + t] - v_ref
  weights[3] = 25; // [delta_start + t]
  weights[4] = 1; // [a_start + t]
  weights[5] = 500;//6000; // [delta_start + t + 1] - [delta_start + t]
  weights[6] = 0.5; // [a_start + t + 1] - [a_start + t]
  weights[7] = 6000; // [delta_start + t] * [v_start + t]
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  size_t n_state = state.size();
  const size_t n_actuators = 2;
  const size_t n_vars = n_state*N + n_actuators*(N-1);
  const size_t n_constraints = n_state*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++)
    vars[i] = 0;
  vars[x_start]     = state[0];
  vars[y_start]     = state[1];
  vars[psi_start]   = state[2];
  vars[v_start]     = state[3];
  vars[cte_start]   = state[4];
  vars[epsi_start]  = state[5];


  // TODO: Set lower and upper limits for variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for(size_t i=0; i<delta_start; ++i)
  {
    vars_lowerbound[i] = -1.0e19;//std::numeric_limits<double>::min();
    vars_upperbound[i] = 1.0e19;//std::numeric_limits<double>::max();
  }
  for(size_t i=delta_start; i<a_start; ++i)
  {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] =  0.436332;
  }
  for(size_t i=a_start; i<n_vars; ++i)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }
  constraints_lowerbound[x_start]     = state[0];
  constraints_upperbound[x_start]     = state[0];
  constraints_lowerbound[y_start]     = state[1];
  constraints_upperbound[y_start]     = state[1];
  constraints_lowerbound[psi_start]   = state[2];
  constraints_upperbound[psi_start]   = state[2];
  constraints_lowerbound[v_start]     = state[3];
  constraints_upperbound[v_start]     = state[3];
  constraints_lowerbound[cte_start]   = state[4];
  constraints_upperbound[cte_start]   = state[4];
  constraints_lowerbound[epsi_start]  = state[5];
  constraints_upperbound[epsi_start]  = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  //auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;
  //std::cerr << "Size of solution.x vector: " << solution.x.size() << "\n";

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  //std::cerr << "delta: "  << solution.x[delta_start]  << "\n";
  //std::cerr << "a: "      << solution.x[a_start]      << "\n";
  for(size_t i=0; i<N; ++i)
  {
    x[i] = solution.x[x_start + i];
    y[i] = solution.x[y_start + i];
  }
  int offset = static_cast<int>(delay/dt);
  return {solution.x[delta_start + offset], solution.x[a_start + offset]};
}
