//@detected[atomic]
+gantry_detected(_) : working(Location) & previous_location(Location,Prev) & counterClock
	<- 
		stop_movement; // stops moving and cancel any navigation goals
		.print("Detected the Gantry robot in close proximity, turning Clockwise"); //selecting a different table.");
		.wait(1500); // waits for 3 seconds before resuming
		!work(Prev);
		.wait(1500); // waits for 5 seconds to get away from the Gantry
		-counterClock; // 		.abolish(counterClock);
		.abolish(gantry_detected(_)).
		
//@detected[atomic]
+gantry_detected(_) : working(Location) & next_location(Location,Next) & not counterClock
	<- 
		stop_movement; // stops moving and cancel any navigation goals
		.print("Detected the Gantry robot in close proximity, turning Counterclock"); //selecting a different table.");
		.wait(1500); // waits for 3 seconds before resuming
		!work(Next);
		.wait(1500); // waits for 5 seconds to get away from the Gantry
		+counterClock;
		.abolish(gantry_detected(_)).
		
