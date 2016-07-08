# Matlab_boat_simulation

___________

## How to use the code
___________

1. To launch the simulation : 

	 just launch collision_avoidance_sailing_boat_main.m
___________

2. To setup waypoints :

* Use click_to_pos.m and modify
  - **waypoints.mat** for waypoints
  - **sailing_zone.mat** for the sailing_zone borders
  - **obstacles.mat** for the obstacles. When the matrix is created, it appear in the map folder. You need to move it to the **Waypoint_matrix_and_associated_tools folder**.

  Or change it in the code in collision_avoidance_sailing_boat_main.m in the world variables section.
___________

3. To modify values for the following_line algorithm you need to modify 
them inside the function **follow_line.m**.
___________

4. Explanation of the variables : 

	r    : the radius of the channel where the boat is authorised to tack. (meters)
	rq   : the security radius of obstacles (meters)
	phat : the current objective (coordinates in meters)
	qhat : the list of obstacles (coordinates in meters)
___________

5.  Remarks

- The computation of the sailingZone is only done once in the variable initialization to reduce the computation time.

- There is a lot of input into the different functions, this is normal. This is to be able to modify most of the values from the main. There is no global variables. This is to be able to see what is exactly the input of each function.
	
___________

## Update notes : 
	
08/07/2016 

	 - Upload of the code to a remote repository
	 
	 - Merge with Heading Only Mode code
		Function changed by the heading only mode : 
		 - main
		 - boat_on_collision_course
		 - avoid_mode
		 - calculate_potField
        Use of detect_nearest_obstacle to fix the issue on collisionnedObstacle
        Currently the boat in bearing only mode is sailing through a wall of obstacle

     - Backup version
