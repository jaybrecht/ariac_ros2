/* Initial beliefs and rules */
// The location belief contains the location and the X,Y,Z coordinates
// Positions of the tables are with respect of the initial position of the screen when the simulation starts
// tableBR = table bottom right, tableBL = table bottom left, tableTR = table top right, tableTL = table top left
// safe zone TBD
location(tableBR, X, Y, 0.0).
location(tableBL, X, Y, 0.0).
location(tableTR, X, Y, 0.0).
location(tableTL, X, Y, 0.0).
location(safe, X, Y, 0.0).

// Beliefs that determine where is the next location to work (second parameter) based on the current one (first parameter)
next_location(tableBR,tableTR).
next_location(tableTR,tableTL).
next_location(tableTL,tableBL).
next_location(tableBL,tableBR).

/* Initial goals */

!start.

/* Plans */

+!start : indifferent <- .include("indifferent.asl"); !work(tableBR).
+!start : helpful <- .include("helpful.asl"); !work(tableBR).
+!start : antagonistic <- .include("antagonistic.asl"); !work(tableBR).

// Work pattern plans
+!work(Location) : location(Location, X, Y, Z) 
	<- 
		-working(_);
		+working(Location);
		move(X, Y, Z).


+work_completed : working(Location) & next_location(Location,Next)
	<-
		!work(Next).


// Plan for when gantry is disabled
@disabled[atomic]
+gantry_disabled : working(Location)
	<-
		stop_movement; // stops moving and cancel any navigation goals
		teleport_safe; // ask to be teleported to the safe zone
		.wait(8000); // waits for 8 seconds in the safe zone
		!!work(Location). // resumes work pattern
		
