@detected[atomic]
+gantry_detected(_) : not movingToGantry  //working(Location) & g_position(X, Y, Z) 
	<-
		.print("Detected the Gantry robot in close proximity and moving towards me, I want to mess with it.");
		stop_movement; // stops moving and cancel any navigation goals
		+movingToGantry;
		move_to_gantry;
		.wait(3000). // waits for 3 seconds before resuming

