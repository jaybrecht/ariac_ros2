+gantry_detected(_) : working(Location) & next_location(Location,Prev) 
	<-
		stop_movement; // stops moving and cancel any navigation goals
		.print("Detected the Gantry robot in close proximity and moving towards me, I want to mess with it.");
		.wait(1000);
		//+movingToGantry;
		move_to_gantry;
		.wait(12000); // waits for 12 seconds
		stop_movement; 
		.wait(1000);
		!work(Next);
		.wait(1500). 
