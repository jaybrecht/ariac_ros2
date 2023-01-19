@detected[atomic]
+gantry_detected(_) : working(Location) & previous_location(Location,Prev)
	<- 
		.print("Detected the Gantry robot in close proximity and moving towards me, selecting a different table.");
		stop_movement; // stops moving and cancel any navigation goals
		.wait(3000); // waits for 3 seconds before resuming
		!work(Prev);
		.wait(10000). // waits for 5 seconds to get away from the Gantry
		
