# Part 4 - Analysis and Evaluation

This report answers the questions in the WhatsApp brief for the differential-drive robot and the planar 2R manipulator.

## 4.1 Why dead reckoning drifts

Dead reckoning integrates wheel motion over time, so every small error in wheel
radius, wheel-base measurement, encoder count, or timing becomes a pose error.
The errors accumulate rather than being corrected by an external measurement.
Unequal wheel slip, floor irregularities, backlash, and battery or motor
differences also cause the real robot to move differently from the ideal model.

The simulation does not model wheel slip, encoder quantisation, backlash,
mechanical tolerances, motor dynamics, uneven surfaces, sensor noise, or
communication/timing jitter. It uses exact commanded wheel speeds and a fixed
time step. Increasing `Delta t` makes the numerical integration less accurate:
the position uses the heading at the beginning of each step, so curved motion
is approximated by short straight segments. A smaller `Delta t` reduces this
discretisation error, but cannot remove real-robot modelling errors.

## 4.2 Euler pose update versus exact arc update

For constant body velocity `v` and angular velocity `omega`, the exact update
over a step `Delta t` is

```text
x' = x + (v/omega) [sin(theta + omega*Delta t) - sin(theta)]
y' = y - (v/omega) [cos(theta + omega*Delta t) - cos(theta)]
theta' = theta + omega*Delta t
```

when `omega` is non-zero. For `omega = 0`, the exact motion is

```text
x' = x + v*cos(theta)*Delta t
y' = y + v*sin(theta)*Delta t
theta' = theta
```

The current simulator uses forward Euler:

```text
x' = x + v*cos(theta)*Delta t
y' = y + v*sin(theta)*Delta t
theta' = theta + omega*Delta t
```

Both methods have the same angular update, but Euler replaces the circular arc
with a chord/tangent step. The difference becomes important when
`abs(omega*Delta t)` is not small, especially during fast turns, large time
steps, or long simulations. With the project's `Delta t = 1/60 s`, the error
is usually small for moderate speeds, but exact integration is preferable for
accuracy at little extra cost.

## 4.3 Why a differential-drive robot cannot move sideways

The no-side-slip constraint is

```text
xdot*sin(theta) - ydot*cos(theta) = 0
```

It says that velocity perpendicular to the robot's forward axis is zero.
Therefore, the body can command forward velocity `v` and angular velocity
`omega`, but not an independent lateral velocity. It has three pose
coordinates `(x, y, theta)` but only two instantaneous control inputs, so it
has two degrees of actuation and one nonholonomic constraint. It can still
reach general planar poses by combining forward motion and rotation, but it
cannot translate sideways instantaneously.

## 4.4 Instantaneous centre of rotation

Let the wheel angular speeds be `wL` and `wR`, wheel radius be `r`, and wheel
separation be `L`. The body velocities are

```text
v = r/2 * (wR + wL)
omega = r/L * (wR - wL)
```

For `omega != 0`, the instantaneous centre of rotation lies on the robot's
sideways axis at signed distance

```text
R = v/omega = L/2 * (wR + wL)/(wR - wL)
```

from the midpoint between the wheels. Thus:

- `wR = wL`: `omega = 0`, so the centre is at infinity and the robot moves
  straight.
- `wR/wL > 0`: both wheels move in the same general direction and the robot
  follows an arc; the centre is on one side, with the side determined by which
  wheel is faster.
- `wR/wL < 0`: the wheels turn in opposite directions and the centre lies
  between the wheels or on the opposite side.
- `wR = -wL`: `v = 0`, so `R = 0`; the robot turns in place.

## 4.5 Evaluation of the go-to-goal controller

The controller computes distance `rho` and heading error `alpha`, then uses

```text
v = K_rho*rho
omega = K_alpha*alpha
```

with speed and wheel-speed saturation. It generally reaches the goal because
it first turns toward the target and then drives toward it. Near the goal,
the distance-based speed decreases, which helps avoid overshoot.

If `alpha` is approximately `+/- pi`, the target is behind the robot. The
controller commands a large turn, and wheel saturation can make the turn
slower or distort the requested `(v, omega)`. The robot may initially move
slightly while turning, and noisy or discontinuous heading changes can cause
oscillation. A higher `K_alpha` gives faster heading correction but can make
the motion sharper and less smooth; a lower value is smoother but slower and
may allow a larger tracking error. A good compromise is to reduce forward
speed when the heading error is large, for example
`v = K_rho*rho*cos(alpha)` with a non-negative clamp, while retaining a
bounded `omega`. This makes the robot turn before driving strongly toward the
goal and reduces overshoot.

## 4.6 Joint-space versus Cartesian straight-line arm motion

In joint-space motion, `q1` and `q2` are interpolated directly. It is simple,
smooth, and naturally respects joint coordinates, but the end effector
usually follows a curved path in Cartesian space. This is suitable for
pick-and-place when the exact path between the objects is not important and
joint limits or simple timing are the priority.

In Cartesian motion, the desired end-effector point is interpolated along a
straight line and inverse kinematics is solved at each time step. This gives
controlled tool-point geometry and is preferable for welding, cutting,
painting, or any task requiring a straight tool path. Its disadvantages are
repeated IK computation, possible branch changes, workspace limits, and high
joint speeds near singularities.

## 4.7 Joint speeds near a Cartesian singularity

For a planar 2R arm, the Jacobian determinant is

```text
det(J) = L1*L2*sin(q2)
```

The arm is singular when `q2 = 0` or `q2 = +/-pi`, where the links are
collinear. Near such a configuration, a modest Cartesian velocity can require
very large joint velocities because the Jacobian becomes nearly non-invertible.
The Cartesian path may therefore produce spikes, exceed motor limits, or become
impossible to track. A practical system should slow down near low
`abs(det(J))`, enforce joint-speed limits, and use damped-least-squares IK or
re-plan the path.

## 4.8 Comparing the two robots

| Property | Differential-drive robot | Planar 2R arm |
|---|---|---|
| Configuration | `(x, y, theta)` | `(q1, q2)` |
| Degrees of freedom | 3 pose coordinates, 2 controls | 2 independent joint coordinates |
| Main constraint | No instantaneous lateral velocity | Planar link geometry and joint/workspace limits |
| Forward kinematics | Wheel speeds to `(v, omega)` and pose integration | Joint angles to end-effector position |
| Inverse solution | Closed-form wheel IK from `(v, omega)` | Closed-form analytic IK with elbow-up/down branches |
| Uniqueness | Wheel speeds are unique for a specified `(v, omega)` | A reachable point generally has two solutions; some points have one or none |
| Controller role | Go-to-goal controller generates `(v, omega)`; IK converts it to wheels | Cartesian path generator repeatedly calls analytic IK |

The differential-drive robot's inverse kinematics is a direct algebraic mapping
from desired body velocity to wheel velocities, but its path and final pose
are produced by a controller and integration. The arm has a closed-form
geometric IK, but the solution is not necessarily unique because of elbow-up
and elbow-down configurations. Both systems are limited by singular or
unreachable configurations: the robot cannot command lateral velocity, while
the arm cannot reach outside its annular workspace or move reliably through a
singularity.
