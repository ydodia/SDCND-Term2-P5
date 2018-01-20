# Model Predictive Control | Term 2 P5

---

This project used the Model Predictive Control (MPC) model to drive a simulated car around a simulated track. The MPC framework frames the problem as an optimization problem where we look for the lowest cost fitted path along a trajectory.

### The Model

##### Polynomial Fitting

For each iteration of the model, the waypoints are given in global coordinates and we convert them to vehicle coordinates. Then the set of transformed`x-y` points is used to perform a **3rd degree** polynomial fit via the provided `polyfit` function. The resulting coefficients are used to map the low-cost vehicle trajectory from the MPC model solver (described below).

##### Latency

Next the model incorporates a latency of 100ms. For each input or time step from the simulator, we update the state with latency. Since all state information is from the car's reference frame, without any latency we'd have a starting `x-y` position of `(0,0)` as well as `psi` of `0`. Given a time delay of 100ms, we apply the below state update equations to get our updated state.

* `psi_t+1   = v_t*steer_angle/Lf*delay`
* `x_t+1 = x_t + v_t*cos(psi)*delay`
* `y_t+1 = y_t + v_t*sin(psi)*delay`
* `v_t = a_t*delay`
* `cte+_t+1 = cte_t  + v_t*sin(epsi_t)*delay`
* `epsi_t+1 = epsi_t + v_t*steer_angle/Lf*delay`

`steer_angle` is the steering angle.
`Lf` is the distance from the front wheels to the vehicle center of gravity.
`v_t` is the velocity at time `t`.
`cte_t` is the CTE at time `t`.
`epsi_t` is the error-psi at time `t`.

We then feed in the following state vector to the **Ipopt** solver: `x, y, psi, v, cte, epsi`. The solver gives us back the path with the least cost depending on our custom defined cost function. My incorporated costs are below with descriptions.

* **CTE** - Penalize distance/drift from center of track.
* **ePsi** - Penalize error-psi; important b/c it pertains to car's heading.
* **Velocity** - Keep car moving close to a set reference velocity.
* **Steering Angle** - Penalize sharp, sudden turns.
* **Acceleration** - Don't really need to penalize sudden acceleration.
* **Steering Angle delta** - Smooth out turns.
* **Acceleration delta** - Smooth out throttling.
* **Steering Angle * Velocity** - Penalize hard turning at high speeds.

I chose a much higher weight `(6000)` for **Steering Angle * Velocity** relative to the others. The next two highest weights were for **Steering Angle delta** and **CTE**.

##### Hyperparameters
I settled on values of `N=16` and `dt=0.1` hyperparameters from experimentation. The faster the the vehicle's speed, the smaller the `dt`. My values resuls in a 'look-ahead' time of `1.6s`. For a vehicle moving at `100mph` (`44.7m/s`) that equates to a distance of `71.52m`. This allows for enough time to anticipate the path's landscape as well as not waste resources computing a path relatively farther in the future which has less marginal value to the vehicle's near-term decision making.

I had attempted `N=20` and `dt=0.1` as well as `N=22` but these seemed to `look ahead` a bit too much and could overshoot the turn on sharp turns. I also played around with the `dt` parameter trying `dt=0.125` but felt that at higher speeds I'd need faster actuations. A value of `dt=0.075` was a little too 'fast' for responses and would require frequent computations for a fixed `T=N*dt` which in my case was `T=1.6s`. In the end, this `T` proved to be a good balance for a wide speed range (from 45mph to 100mph).

### Results
The veichle averages `75mph` around the track and successfully completes multiple consecutive laps. The mean-square-error CTE is `~0.6`.

<p align="center">
  <img width="290" height="175" src="./lap.gif">
</p>
      




