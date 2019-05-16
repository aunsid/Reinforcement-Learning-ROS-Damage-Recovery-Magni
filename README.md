# Damage-Recovery-Q-Learning

As robot autonomy continues to improve, a growing number of robots are being deployed into real-world situations where there is a high probability of hardware damage that can prevent them from performing their designed tasks. Instead of trying to diagnose the damage itself, the robots are adapting to their damage using a trial-and-error approach called Reinforcement Learning (RL). For this Damage Recovery project, we were given the Magni Robot, to implement RL using C++ and ROS to “fix” a damaged motor, wheel, or leg depending on the application. Our RL algorithm would learn to correct for “damage” using the position and orientation of the robot as inputs and outputting a prescribed motor velocity, linear in X and angular in Z. Our goals for our RL algorithm was to stay as close to the straight line as possible and increase the learning rate for the speed of convergence.




## Dependencies


* `ROS-Kinetic`



## Run Instructions

```bash
mkdir -p catkin_ws/src
git clone 
```
For training/testing change line 89 in rlmagniV3.cpp

```bash
cd catkin_ws/src
catkin_make
source devel/setup.bash
roslaunch magni_bringup base.launch
rosrun rl expfast
```
---
## Algorithm

<figure>
 <img src="./Images/basicQlearning.png" width="712" alt="Combined Image" />
 <figcaption>
 <p></p> 
 </figcaption>
</figure>

State Calculation

<figure>
 <img src="./Images/state.png" width="712" alt="Combined Image" />
 <figcaption>
 <p></p> 
 </figcaption>
</figure>

 
## Optimizations

<figure>
 <img src="./Images/optimization.png" width="712" alt="Combined Image" />
 <figcaption>
 <p></p> 
 </figcaption>
</figure>



## Results

<figure>
 <img src="./Images/trainTest.png" width="712" alt="Combined Image" />
 <figcaption>
 <p></p> 
 </figcaption>
</figure>


