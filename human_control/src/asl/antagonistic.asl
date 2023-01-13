+gantry_detected(_) : true  //working(Location) & g_position(X, Y, Z) 
	<-
		.print("Detected the Gantry robot in close proximity and moving towards me, I want to mess with it.");
		stop_movement; // stops moving and cancel any navigation goals
		.wait(5000). // DEBUG: waits for 5 seconds
		//.print("Now moving towards Gantry: " + X +", " + Y + ", " + Z);
		// move_to_gantry.

