/* Initial beliefs and rules */
// The location belief contains the location and the X,Y,Z coordinates
// Positions of the tables are with respect of the initial position of the screen when the simulation starts
// station1 = table bottom right, station2 = table top right, station4 = table top left, station3 = table bottom left
location(station1, -5.75, 4.5, 0.0).   //   -5.75,  3.0, 0.0
location(station2, -10.85, 4.5, 0.0).  //  -11.85, -3.0, 0.0
location(station4, -10.85, -4.5, 0.0). //  -11.85, -3.0, 0.0
location(station3, -5.75, -4.5, 0.0).

// safe zone, not being used by the agent at the moment
location(safe, -15.29, -10.04, 0.0).

// Beliefs that determine where is the next location to work (second parameter) based on the current one (first parameter)
next_location(station1,station2).
next_location(station2,station4).
next_location(station4,station3).
next_location(station3,station1).

/* Initial goals */

!start.

/* Plans */

+!start : indifferent <- .include("indifferent.asl"); !work(station4).
//+!start : helpful <- .include("helpful.asl"); !work(station4).
//+!start : antagonistic <- .include("antagonistic.asl"); !work(station4).

// Work pattern plans
+!work(Location) : location(Location, X, Y, Z) 
	<- 
		-working(_);
		+working(Location);
		.print("Now moving to: ", Location);
		move(X, Y, Z).


+work_completed(_) : working(Location) & next_location(Location,Next)
	<-
		.print("Work completed at ", Location);
		!work(Next).


// Plan for when gantry is disabled
@disabled[atomic]
+gantry_disabled : working(Location)
	<-
		stop_movement; // stops moving and cancel any navigation goals
		teleport_safe; // ask to be teleported to the safe zone
		.wait(8000); // waits for 8 seconds in the safe zone
		!!work(Location). // resumes work pattern
		
