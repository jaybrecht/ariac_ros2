+gantry_detected : working(Location) & next_location(Location,Next)
	<- 
		.print("Detected the Gantry robot in close proximity and moving towards me, selecting a different table.");
		stop_movement; // stops moving and cancel any navigation goals
		!work(Next).
