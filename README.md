# Driving simulator Proportional–Integral–Derivative (PID) controller

This control loop feedback mechanism in this project continuously calculates a cross track error (CTE), the distance between of the car from the middle of the road, and applies a correction using proportional, integral, and differential terms. Tunning these three hyper-parameters allows for successfully optimized steering and for a vehicle to stay on a simulated track.

## Proportional

The proportional term accounts for present values of the CTE. A large positive CTE will result in a control output that is proportionally large and positive. With the proportional parameter set high a vehicle may respond to sharp corners but have a tendency to overshoot the middle of the road and oscillate across the middle of the road sharply. The vehicle will cross the middle of road as it will receive a low CTE and will provide a low control response.

## Integral

The integral term accounts for past values of the CTE and integrates them over time in the controller. In that way it accounts for bias such as wind or a misalignment in steering or when the proportional term is not providing a sufficient control response. The integral term will reduce under steering over time.

## Differential

The differential term accounts for future values of the CTE and counteracts the tendency to overshoot the middle of the road. An appropriate weighted differential component will dampen a cars steering and prevent it from quite reaching the center of the road while too large a value will prevent the car from moving much at all.

## Tuning

I first attempted manual tunning at low speeds. At 10 mph I set the P to a 0.1 where it could stay on the track until almost the first corner and then I increased the D until close to 0.1 where it could stay on the track for an entire loop. I left the I = 0 as I didn't anticipate any error from steering or other factors to be a consideration in the simulator.

After this I used the twiddle algorithm to find optimal variables at different speeds. Anytime the CTE was above 2.5 I reset the vehicle back to the start and added that to the total restarts. At low speeds the twiddle alogrithm worked sufficiently. 


[Video] (https://youtu.be/xfZ_cqbaJ1c)

https://en.wikipedia.org/wiki/PID_controller#Manual_tuning
# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
