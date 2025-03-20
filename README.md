# Aircraft Guidance and Control
This program controls aircraft in the popular aerospace simulator Kerbal Space Program using the krpc Python library. The algorithm utilizes numeric simulation methods and PID controllers to achieve smooth, level flight and several maneuvers (i.e. take-off, turning, landing)


## Objective
The objective of this project was to develop an autonomous aircraft control algorithm in Kerbal Space Program. The directives required were:
- Smooth, level flight
- Completing a series of continuous maneuvers in the following order:
    - Take-off
    - Cruise at altitude for some distance
    - 180 degree banked turn
    - landing back on the runway
 
In its current state, this algorithm satisfies these objectives

## Numeric Methods
The control algorithm uses numeric methods to make incremental control updates to the aircraft. The class attribute: `self.t = time_step` is an infinitesimal time interval between control updates to the aircraft.

### Target velocities
At the start of each simulation step, a target velocity for several **quantities** is calculated via a quadratic relationship between how much the **quantity** differs from its target (i.e. `abs(altitude - target_altitude)`) and the target velocity of how the aircraft should move to correct for this difference. For example, the vertical velocity the aircraft should attain to correct how its altitude is off from what is desired will depend quadratically on how much its altitude differs from what is desired. (example **quantities**: altitude, roll, etc)

### Time derivatives
Next, an array of instantaneous time derivatives is calculated for several **quantities**, with the attribute `self.t` as the time differential.

### Controller
At the end of the simulation step, the PID controller method is called. With a **quantity's** target velocity and array of time derivatives, a boolean statement decides to make a control adjustment in one of two directions or do nothing. Should a control adjustment be made, the magnitude of the control adjustment is also calculated via a quadratic dependence on the difference in the **quantity's** velocity and target velocity.

## Generalizations - Future Iterations
There are very few, if any, aircraft specific constants in the aircraft's class attributes. A soft goal while approaching the project was creating a semi aircraft-agnostic autonomous control algorithm. What was compelling about this approach was that an autonomous control algorithm could be one which can pilot many aircraft of various aerodynamic properties. This approach is by no means an engineering best-practice, but merely a thought experiment which partially inspired the project.

An idea for a future iteration of this project is scaling to multiple autonomous aircraft flying with some inter-aircraft control algorithm. This opens up the possibility for autonomous aircraft formations, aerial fuel-docking, etc.



  

