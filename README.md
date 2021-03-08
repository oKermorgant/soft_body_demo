# Demo of interaction betweem physics and visuals for Gazebo plugins

# Model

The model is a simple mass on top of a spring. The spring shape is reconstructed (in the physics part) and sent to the visual part. 

Mechanics are not real at all, they are just decoupled mass-spring to make the system oscillate.

# Built libraries

Four libraries are built in this package. 

* `soft_body_shape`: tools to describe the shape of the spring (here: spline) and to communicate from physics to visual.

* `soft_body_visual_plugin`: Gazebo visual plugin library. 
 * Depends on `soft_body_shape` in order to receive the shape.
 
* `soft_body_physics`: Gazebo-agnostic physics library. Receives the current state of the model and output the wrench to add. 
 - Depends on `soft_body_shape` in order to send the shape. 
 - Depends on `Eigen` for the math. could be another math library
 
* `soft_body_physics_plugin`: Gazebo model plugin
 - Depends on `soft_body_physics` in order to call the custom physics
 
 
