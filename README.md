# Demo of interaction betweem physics and visuals for Gazebo plugins

# Model

The model is a simple mass on top of a spring (`launch/mass_spring.sdf.xacro`). The spring shape is reconstructed (in the physics part) and sent to the visual part. 

SDF format has to be used in order to have visual plugin working.

# Built libraries

Four libraries are built in this package. 

* `soft_body_shape`: tools to describe the shape of the spring (here: spline) and to communicate from physics to visual.
   * Communication is done with  Zero-MQ through inter-process communication (fast / easy to write).

* `soft_body_visual_plugin`: Gazebo visual plugin library. 
   * Depends on `soft_body_shape` in order to receive the shape.
 
* `soft_body_physics`: Gazebo-agnostic physics library. Receives the current state of the model and output the wrench to add. 
   * Depends on `soft_body_shape` in order to send the shape. 
   * Depends on `Eigen` for the math. could be another math library
   * Mechanics are not real at all, they are just decoupled mass-spring to make the system oscillate.
 
* `soft_body_physics_plugin`: Gazebo model plugin
   * Depends on `soft_body_physics` in order to call the custom physics
 
# Executables

The node `physics_node` is also built to process the physics. It communicates with Gazebo through services and uses the `soft_body_physics` to do the computations (and thus send the shape to the visual plugin).

# Launch files

* `gazebo.launch` just runs Gazebo
* `launch/spawn.launch` spawns the model. If `enable_physics_plugin` is `True` then physics are done through the model plugin. Otherwise they are done through an external node. 

# Pros and cons of plugin vs external node

## Advantages of running physics as a plugin

* The custom physics is ensured to be called at each Gazebo simulation iteration (by default: 1 kHz)
* If your custom physics computations take too long, Gazebo will slow down the simulation time to adapt to it
* No need to use ROS services to transfer states and wrenches with Gazebo, which is tedious to write and induce delays
* Actually no need to use ROS at all

## Advantages of a running physics as an external node

* It is currently impossible to reload a plugin in Gazebo, even after it was recompiled. So while developping custom physics, you would have to stop / restart Gazebo each time you want to test new code. It will already be the case for the visual plugin, you do not want that for the physics.
* The main advantage is thus that you can stop / restart the node (ie your custom physics) when you want. In the example, the node resets the simulation so you can just leave Gazebo running and the node will put everything back in place when it starts.
* You can also publish other things for logging purposes

# Advised workflow

* Decide what shape should be displayed
* Write a dummy physics code that just publishes this shape in order to develop the visual plugin (you'll have to restart Gazebo many times for that)
* Work on the physics code with external node approach to ease testing / debugging. If the computation is too high, reduce your step size. The consequence is just that your physics will use constant wrench during several Gazebo iterations, which is not so bad.
* Once it is done, write the corresponding model plugin. 

## Additional notes

* coordinates in the visual plugin are relative to the visual shape from the model. The example show how to get the inverse scale and size the shape accordingly.
