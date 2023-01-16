import jason.asSyntax.*;
import jason.environment.*;
import java.util.logging.*;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.geometry_msgs.Twist;
import ros.msgs.geometry_msgs.Vector3;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.msgs.ariac_msgs.Snapshot;
import ros.msgs.ariac_msgs.TimedPose;
//import ros.msgs.move_base_msgs.MoveBaseActionResult;
import ros.tools.MessageUnpacker;
import com.fasterxml.jackson.databind.JsonNode;

public class RosEnv extends Environment {

    private Logger logger = Logger.getLogger("ariac_env."+RosEnv.class.getName());
    
    final int gantry_detection = 10;

	int cont = 0;
	int detcont = 0;

	double gpX = 0.0;
	double gpY = 0.0;
	double gpZ = 0.0;
    
    RosBridge bridge = new RosBridge();

    /** Called before the MAS execution with the args informed in .mas2j */
    @Override
    public void init(String[] args) {
        super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");
		
		/* Subscriber for getting the distance between the Gantry and the human */
		bridge.subscribe(SubscriptionRequestMsg.generate("/snapshot")
				.setType("ariac_msgs/Snapshot") // added to java_rosbridge_all
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {

				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<Snapshot> unpacker = new MessageUnpacker<Snapshot>(Snapshot.class);
					Snapshot msg = unpacker.unpackRosMessage(data);
					if (msg.distance_robot_human_operator <= gantry_detection) {
						//clearPercepts("human");
						//logger.info("I see the Gantry robot in less than " + msg.distance_robot_human_operator +" meters: gantry_detected");
						Literal gDetectedLit = new LiteralImpl("gantry_detected"); //movebase_result");
						gDetectedLit.addTerm(new NumberTermImpl(detcont++)); //movebase_result.addTerm(new NumberTermImpl(msg.status.status));
						addPercept("human",gDetectedLit); 
						//addPercept("human",Literal.parseLiteral("gantry_detected")); 
					}
//					logger.info(msg.time+"");
//					logger.info(msg.human_operator_speed+"");
//					logger.info(msg.robot_speed+"");
//					logger.info(msg.distance_robot_human_operator+"");
				}
			}
	);
	
		//New func by Angelo
	/* Subscriber for getting the current Gantry position */
	bridge.subscribe(SubscriptionRequestMsg.generate("/monitor/robot/pose")
		.setType("ariac_msgs/TimedPose") // added to java_rosbridge_all
		.setThrottleRate(1) 
		.setQueueLength(1),
	new RosListenDelegate() {

		public void receive(JsonNode data, String stringRep) {
			//logger.info("Robot pose received, creating belief: gantry_position");
			MessageUnpacker<TimedPose> unpacker = new MessageUnpacker<TimedPose>(TimedPose.class);
			TimedPose msg = unpacker.unpackRosMessage(data);
			//clearPercepts("human");
			//Literal gantry_position = new LiteralImpl("gantry_position");
			// gantry_position.addTerm(new NumberTermImpl(msg.value.position.x)); // this is to check, depending on the type of msg.value
			// gantry_position.addTerm(new NumberTermImpl(msg.value.position.y)); // this is to check, depending on the type of msg.value
			// gantry_position.addTerm(new NumberTermImpl(msg.value.position.z)); // this is to check, depending on the type of msg.value
			gpX = msg.value.position.x;
			gpY = msg.value.position.y;
			gpZ = msg.value.position.z;
			//addPercept("human", gantry_position);
		}
	}
	);

		/* Subscriber for getting the information that the Gantry has been disabled. Note that the topic is made up, it should be replaced with the correct one later */
		bridge.subscribe(SubscriptionRequestMsg.generate("/gantry_disabled")
				.setType("std_msgs/Bool")
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<PrimitiveMsg<Boolean>> unpacker = new MessageUnpacker<PrimitiveMsg<Boolean>>(PrimitiveMsg.class);
					PrimitiveMsg<Boolean> msg = unpacker.unpackRosMessage(data);
					if (msg.data){
						clearPercepts("human");
						logger.info("Gantry has been disabled!");
						addPercept("human",Literal.parseLiteral("gantry_disabled"));
					}
				}
			}
	); 
	
		/* Subscriber for move_base result. Note that this will change in ROS2, needs experimentation */		
		bridge.subscribe(SubscriptionRequestMsg.generate("/move_base_result")
				.setType("std_msgs/Bool")
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<PrimitiveMsg<Boolean>> unpacker = new MessageUnpacker<PrimitiveMsg<Boolean>>(PrimitiveMsg.class);
					PrimitiveMsg<Boolean> msg = unpacker.unpackRosMessage(data);
					clearPercepts("human");
//					System.out.println("Frame id: "+msg.header.frame_id);
//					System.out.println("Stamp sec: "+msg.header.stamp.secs);
//					System.out.println("Seq: "+msg.header.seq);
//					System.out.println("Goal: "+msg.status.goal_id.id);
//					System.out.println("Stamp sec: "+msg.status.goal_id.stamp.secs);
//					System.out.println("Status: "+msg.status.status);
//					System.out.println("Text: "+msg.status.text);
//					System.out.println();
					logger.info("Human reached waypoint	!");
					Literal movebase_result = new LiteralImpl("work_completed"); //movebase_result");
					movebase_result.addTerm(new NumberTermImpl(cont++)); //movebase_result.addTerm(new NumberTermImpl(msg.status.status));
					logger.info("cont: "+cont);
					addPercept("human", movebase_result);
					//addPercept("human",Literal.parseLiteral("work_completed"));
				}
			}
	    );
		
    }
    @Override
    public boolean executeAction(String agName, Structure act) {
		if (act.getFunctor().equals("move")) {
			NumberTerm lx = (NumberTerm) act.getTerm(0);
			NumberTerm ly = (NumberTerm) act.getTerm(1);
			NumberTerm lz = (NumberTerm) act.getTerm(2);
			try{
				//logger.info("Move requested by agent, will now send topic");
				move(lx.solve(),ly.solve(),lz.solve());
			} catch(Exception e) {
				e.printStackTrace();
			}
		}
		else if (act.getFunctor().equals("stop_movement")) { 
			stop_moving();
		}
		else if (act.getFunctor().equals("teleport_safe")) { 
			teleport();
		}
		else if (act.getFunctor().equals("move_to_gantry")) { 
			move_to_gantry(); 
		}
		else {
			logger.info("PROBLEM: requested: "+act.getFunctor()+", but not implemented!");
		}
        informAgsEnvironmentChanged();
        return true; // the action was executed with success
	//return super.executeAction(agName, act);
    }
    
	/*public void hello_ros() {
		
		for(int i = 0; i < 100; i++) {
			pub.publish(new PrimitiveMsg<String>("hello from Jason " + i));
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}*/
	
	public void move(double x, double y, double z) {
		Publisher move_base = new Publisher("/jason_to_move_base", "geometry_msgs/Vector3", bridge);		
		move_base.publish(new Vector3(x,y,z));
	}
	
	// This method also needs to cancel any ongoing move base goals
	public void stop_moving() {
		Publisher move_base = new Publisher("/jason_stop_human", "geometry_msgs/Vector3", bridge);		
		move_base.publish(new Vector3(0.0, 0.0, 0.0)); //LB: could fix: reimplement without parameters
	}
	
	// 2 do: Should call a service by sending a message to a topic and having a Python script reading it to send the service request
	public void teleport() { // ros2 service call /ariac/teleport_human ariac_msgs/srv/TeleportHuman
		Publisher teleport_h = new Publisher("jason_teleport_human", "std_msgs/Bool", bridge);	
		logger.info("RosEnv: executing Teleport");	
		teleport_h.publish(new Boolean(true)); //new Vector3(-15.0, -10.0, 0.0));		
	}

	// Fix: Should call a service by sending a message to a topic and having a Python script reading it to send the service request
	public void move_to_gantry() {
		Publisher stop = new Publisher("/jason_stop_human", "geometry_msgs/Vector3", bridge);		
		stop.publish(new Vector3(0.0, 0.0, 0.0)); //LB: could fix: reimplement without parameters
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

		Publisher move_base = new Publisher("/jason_to_move_base", "geometry_msgs/Vector3", bridge);		
		move_base.publish(new Vector3(gpX, gpY, gpZ));  //move_base.publish(new Vector3(-8.0, 0.0, 0.0));	
		logger.info("Move to gantry requesed");	
	}

    /** Called before the end of MAS execution */
    @Override
    public void stop() {
        super.stop();
    }
 }
