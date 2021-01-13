# Multi-robot navigation in cluttered and dynamic environment

In this project, we implemented a non-linear aggregation algorithm using proximal control for obstacle avoidance, and infrared communication for both heading alignment and swarm cohesion. This algorithm provides a consistent flocking navigation, given good communication range and sensors stability. It still exhibits a good consistency on real Epuck robots that have a very short communication range and noisy sensors.

For more information see [report](https://github.com/prodp/DISProject/blob/main/report.pdf).

## Code Organisation

* webots
	* webots/controllers
		* flock_controller_compass: controller implementation with optimals conditions in simulation - large communication range and compass node for heading. Implemented for a flock size of five robots.
		* flock_controller: controller implementation with large communication range and odometry. Implemented for a flock size of five robots.
		* flock_controller_calib: controller implementation with non-optimal conditions to match hardware limitations and give an estimation of the reality gap - odometry and communication range of 0.15 meters. Implemented for a flock size of three robots.
		
		* performance_estimation: supervisor that writes the performances of the flock in the matlab folder
		
		* pso_controller: controller for pso optimisation and optimals conditions
		* pso_super: supervisor for pso optimisation and optimal condition
		
		* pso_controller_calib: controller for pso optimisation and non-optimal conditions
		* pso_super_calib: supervisor for pso optimisation and non-optimal conditions
	* webots/worlds
		* the two worlds provided adapted for the different controllers implemented
* epuck_libIRcom
	* epuck_libIRcom/e-puck/prog/test/: hardware controller implementation
	* epuck_libIRcom/e-puck/src: provided library for hardware
* matlab
	* matlab/evaluate.m: matlab script to plot the results obtained with the performance_estimation supervisor
	* matlab/res: all the obtained performances details in the report
* docs
	* docs/metrics.pdf: the metrics implemented by performance_estimation supervisor
	* docs/Self-organized_flocking_in_mobile_robot_swarms.pdf: the article that described the initial algortihm that we adapted for the controllers.
	* docs/report.pdf: our final report that explains the implementation and the results obtained


## Contributors
[Martin Chatton] , [Valentin Gabeff], [Diana Petrescu]
