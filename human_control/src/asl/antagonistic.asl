+gantry_detected : working(Location) & g_position(X, Y, Z) 
	<-
		.print("Detected the Gantry robot in close proximity and moving towards me, I want to mess with it.");
		//teleport_safe; //LB: created for testing purposes
		stop_movement; // stops moving and cancel any navigation goals
		.wait(8000); // waits for 8 seconds in the safe zone
		move(X, Y, Z);
		!work(Location).



