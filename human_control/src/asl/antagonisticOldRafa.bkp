+gantry_detected : working(Location) & next_location(Location,Next)
	<- 
		.print("Detected the Gantry robot in close proximity and moving towards me, I want to mess with it.");
		stop_movement; // stops moving and cancel any navigation goals
		move_to_gantry.

