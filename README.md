# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program - Term 3, Project 1

Anand Dinakar

---

## Goal
We are given a simulated environment with a 6-lane highway with traffic moving in either direction. The goal is to safely navigate around the virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.

The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 

The car should be able to make one complete loop around the 6946m highway. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

[Simulator download](https://github.com/udacity/self-driving-car-sim/releases).

### System interface

The simulator invokes the plath planner approximately every 40 or 50 ms and supplies the car's localization and sensor fusion data.

The path planner is expected to return a path which is a set of points (in map coordinate space) that the car will drive over. The car's perfect controller will visit each point exactly 20ms apart. So to drive faster, the points must be spaced apart and closer together to slow down.

---

## Smooth Trajectory
The first goal I set myself is to generate a smooth trajectory that keeps the car on the center lane, at fixed speed and completely ignoring other cars !
In other words, keep d fixed at 6.0 (middle of center lane); vary only frenet s, velocity (us) and accelaration (as).

Here's a video of the result:


The main challenge was to ensure continuity in the path points, speed and accelaration between calls to the path planner.

The resulting workflow was quite simple:

1. Set initial configuration to (s=0, us=0, as=0)

2. Calculate the final configuration using laws of motion formula:
    sf = (us * t) + 0.5 * as * t^2
    vf = us + (as * t)
    Here, accelaration (a) is computed from how quicklt the car can reach the speed limit. I set the car to reach 20m/s in 5 seconds - pretty good car !
    a = SPEED_LIMIT/TIME_TO_MAX
    The value for time (t) is computed to be the end of the trajectory. I set it to be 2 seconds out in the future (100 path points, 0.02ms each).
    But regardless of the accelaration used to compute sf and vf, we set the target configuration to end with a 0-accelartion, this leaves us with a better jerk minimizing trajectory.

3. Generate the coefficients for a Jerk-minimizing trajectory

4. Generate map-coordinates for some key points along this trajectory

5. Use these points to create a spline curve.

6. Interpolate these key points using the spline - this generates a smooth curve.

7. In the next iteration, calculate the initial configuration from the JMT we already have, give the amount of time that has elapsed between the calls. Then repeat from step 2.


Vidoe of trajectory: Staying in lane with constant speed, ignoring other cars

[![Stay in lane with constant speed](https://img.youtube.com/vi/gX6t7NRcsgY/1.jpg)](https://www.youtube.com/watch?v=gX6t7NRcsgY)



## Following cars and staying in lane
Once a smooth, continuous trajectory was achieved, I focussed on slowing down to keep a safe following distance and avoid crashing into cars ahead of us in the same lane.

The steps to realize this was quite simple using the sensor fusion data provided by the simulator:

1. Determine the car directly in front of us (and closest).

2. Calculate the speed of the car in frenet space.

3. Use that cars speed as our final configuration velocity (vf) for trajectory generation.

4. Calculate the final configuration frenet-s coordinate by simply subtracting a safe-following-distance value. I set this to 10m.

5. The tracjectory generated using this final configuration will ensure we are follow the car in front.


## Finite State Machine
To be able to plan lane switches, and excecute them cleanly, I implemented a state machine with the following states:

1. Keep in lane
    - In this state, the car will either speed up to the speed limit (if there is no car in front of it), or, it will follow the car in front.

2. Look to switch lanes:
    - In this state, the car will look for opportunities to switch lanes, but continue staying in the same lane.

3. Switch right:
    - Once the car has decided to switch to the lane to the right (moving from left-most to the center, or center to the right-most), it will enter this state to execute the transition.

4. Switch left:
    - Similar to the above, in this state, the car executes a transition to the lane to the left of the current lane.

I numbered lanes 0 (left-most), 1 (center) and 2 (right), so that Frenet-s coordinates can be computed as d = 2 + (l*4) to be 2, 6 and 10 respectively.


### When is a good time to switch lanes ?
Then the car is looking to switch lanees, it uses the following hueristic to determine if it is safe to transition:

1. The lane should not have a car that is too close to our current Frenet s-value (less than safe following distance of 10m).

2. If a car is slightly over the safe-following distance, it should not be much slower than us - so that we don't collide when transitioning.

3. There should be no car that is behind us (in Frenet-s terms) but faster than us.

This is implemented in the IsSafeToSwitch function.

### Which lane to switch to ?
When the car is in the center lane and looking to switch, it will prefer switching to the left lane over switching to the right lane. But if the left lane is blocked, and the right is clear, it will initiate a switch to the right lane.

When the car is in either left-most or right-most lanes, obviously, there is only one option - the center lane.

If lane transitions are not possible, it executes the stay in lane behavior, waiting for an opportunity in the future.

Passing car on left:

[![Passing car](https://img.youtube.com/vi/xLesBk8xtTw/1.jpg)](https://www.youtube.com/watch?v=xLesBk8xtTw)


Driving for a mile:
https://youtu.be/WwzDpZ_d0AQ

[![1 Mile Lap](https://img.youtube.com/vi/WwzDpZ_d0AQ/0.jpg)](https://www.youtube.com/watch?v=WwzDpZ_d0AQ)

---

## Credits

1. The spline generator here is really easy to use - it is a single .hpp file to include in the project:
http://kluge.in-chemnitz.de/opensource/spline/.

2. Udacity's awesome simulator was a lot of fun - especially at the initial stage of this project when we had some spectacular crashes. Wish I could drive like this in New Jersey.
[![Fun Crash](https://img.youtube.com/vi/lqD9JNocn8o/0.jpg)](https://www.youtube.com/watch?v=lqD9JNocn8o)


## Future work

1. I would like to implement cost functions to generate and select one from many candidate trajectories. This could lead to a smarter path planner.

2. I would adapt the lane switch states to look for an abort a lane change in case of sudden changes in traffic (like another car making an abrupt change).



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

