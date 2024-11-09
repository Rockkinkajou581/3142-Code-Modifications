**Gainscheduling.cpp:**
PID controllers are inherently linear controllers. They are essentially modeling growth by the differential equation dx/dt = kp * x + kd * dx/dt if we ignore I. So, they are clearly not invariant under changes in setpoint. If we think about it for a second, if we perfectly tune a 90 degree turn, what’s actually happening? The robot initially is given max power, then when it gets to like 10 or so degrees it starts decreasing faster and faster, proportional to how close it is to the setpoint. Now use this same constant and turn 180 degrees. It’ll do the same thing: the robot will max speed until like 10 degrees (when theta * kp > 127) and then slow down. But, the robot was accelerating for a longer time in the 180 degree turn, and thus has more velocity when it reaches 10 degrees (vf = a * t). Now, all of our perfectly tuned constants for this final part of the turn are off and our robot ends up overshooting. Compare this difference to if we want a highly precise movement for 30 degrees and 180 degrees with the same kP—the 30 degree turn, the robot barely even accelerates but in the 180 degree turn, it’s zooming. This is why we need gain scheduling. We also use this to map timeouts for each setpoint. 

To implement this, I’m used a form of linear interpolation. One of the principle ideas behind this is that as the lim setpoint -> inf, kp -> 0. That makes sense. The farther we are going, the earlier we must start to decelerate to overcome greater velocities attained by the robot due to greater net work done by the wheels. So we can put that into code by just approximating big setpoints to 0.1 or we could have used a bayesian distribution or like an ae^-x^2 function. I wanted to make sure if I tune a setpoint, it’s gonna use those exact constants at that exact point, so I opted for linear interpolation between points. We tune 45, 90, 135, and 180 and linearly interpolate between points to find the setpoints in between. Same for the length. 

**Chaining.cpp**
This is also explained in the notebook, but the idea is that PID settling time can be quite poor, as when the robot reaches very lower velocities, it’s position is very close to the setpoint, so kp * P is small and thus it’s velocity is small, too, and everything is at steady state. This causes wasted settling time. So, to fix this we can use chaining. I basically give the robot an early setpoint like instead of 90, do 87 or instead of 24 in, do 22 in, then do a crossing exit, which I’ll explain down below. Then, we give the robot an exit speed. The exit speed leaves the robot with a velocity, so that it’s not having to reach a full stop just to accelerate entirely again. 

**MoveToPointGhost.cpp**
I wanted to improve the current boomerang function, which is a movetopose function. The idea is that we can get arc control by seeking an intermediary point parallel to the crossing plane. If the robot needed to arc wider, we seek a point wider off of the arc and then seek the setpoint. We can also change this intermediary point mid movement for greater smoothness. This is kinda tenuous to explain further. 

**SlowCurve.cpp**
The idea of this is to write a function for the robot to follow circular arcs (meant for lower speeds like when pushing tribals in over under). The kinematics on a circular arc are much easier (it’s just a matter of keeping left and rights velocities at a certain ratio). Then, we just need to find the given arc, given a coordinate to arrive at, which is actually not a trivial task. Then, I put in a form of odometry correction. 

**Fullpower.cpp**
This was an old over under function. It would decide if the robot had successfully pushed in a triball or not from the motor velocities, and if the triball was stuck, it would leave. 

**Crossing.cpp**
This is a much easier way to do exiting for PID controllers with odom. Instead of just waiting to see if error < 0, we can do a sign check to see if the robot has crossed a given line through the set point. We do this with a dot product, as if it turns negative, it must have crossed. 
