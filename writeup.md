## Model description
### 1. State
The state vehicle state is described by a 6-dimension vector `[x, y, psi, v, cte, epsi]`, where:
* `x` is the position of the vehicle in the x direction
* `y` is the position of the vehicle in the y direction
* `psi` is an angle that denotes the vehicle orientation
* `v` is speed of the vehicle
* `cte` is cross track error
* `epsi` is the orientation error

### 2. Actuator
Actuated input allows us to control the vehicle state. In our case it is the steering angle and the acceleration. (The acceleration is combined throttle (positive values) and the braking pedal (negative values)).

### 3. Update equations

`x[t+1] = x[t] + v[t] * cos(psi[t]) * dt`

`y[t+1] = y[t] + v[t] * sin(psi[t]) * dt`

`psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt`

`v[t+1] = v[t] + a[t] * dt`

`cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt`

`epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt`

Where:
* `delta[t]` is the steering angle at time `t`
* `Lf` measures the distance between the front of the vehicle and its center of gravity, and is equal to `2.67` in this project.
* `f(x[t])` in our case is 3rd order polynomial that describes the reference trajectory
* `psides[t]` is the desired `psi` that is evaluated as the *arctan* of *f'(x[t])*

## Timestep Length and Elapsed Duration
I tried several pairs of `N` and `dt` and ended up with `N=10` and `dt=0.1`, which gives the total duration of 1 second. The other values of `N` I tried were `8`, `12`, `20` and `50`. The bigger the number was the computation was more expensive. Sometimes too large `N` (with the same value of `dt`) was leading to the controller being unstable and car going of the track. The other values of `dt` I tried were `0.05` and `0.2`, but I could achive better results (car driving faster) with `N=10` and `dt=0.1`.

## Preprocessing of the waypoints
I processed the waypoints in a way that they are in the car's coordinate system, so we can assume the vehicle is positioned at `(0, 0)`. Then the waypoints are rotated by `-psi` so we can assume, that the car's orientation `psi=0`.

The waypoints processing is done by the following block of code:
```
for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - px;
    double shift_y = ptsy[i] - py;

    ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
    ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
}
```

## Latency handling
I handled the latency (100ms) by estimating the state (using the model equations) at current time + the latency and using it as the initial state.

So the new initial state was as follows:
```
state << 0 + v * t_latency,         // x (cos(psi) = 1)
    0,                              // y (sin(psi) = 0)
    0 - v * delta / Lf * t_latency  // psi
    v + acceleration * t_latency,   // v
    cte,                            // cte = polyeval(coeffs, 0)
    epsi;                           // epsi = -atan(coeffs[1])
```

## Other - code and materials attribution

In the project implementation I used code from the Udacity walktrough video (https://www.youtube.com/watch?v=bOQuhpz3YfU) and from the Udacity lessons 18 (Vehicle Models) and 19 (Model Predictive Control) of the second term Self-Driving Cars Nanodegree.

