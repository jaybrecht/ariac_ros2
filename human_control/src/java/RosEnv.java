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
//import ros.msgs.move_base_msgs.MoveBaseActionResult;
import ros.tools.MessageUnpacker;
import com.fasterxml.jackson.databind.JsonNode;

public class RosEnv extends Environment {

    private Logger logger = Logger.getLogger("ariac_env."+RosEnv.class.getName());
    
    final int gantry_detection = 15;
    
    RosBridge bridge = new RosBridge();

		// Publishers
		Publisher move_base = new Publisher("/jason_to_move_base", "geometry_msgs/Vector3", bridge);
		Publisher cmd_vel = new Publisher("/cmd_vel", "geometry_msgs/Twist", bridge);
		
    /** Called before the MAS execution with the args informed in .mas2j */
    @Override
    public void init(String[] args) {
        super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");
		
		/* Subscriber for getting the distance between the Gantry and the human */
		bridge.subscribe(SubscriptionRequestMsg.generate("/snapshot")
				.setType("ariac_msgs/Snapshot")
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {

				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<Snapshot> unpacker = new MessageUnpacker<Snapshot>(Snapshot.class);
					Snapshot msg = unpacker.unpackRosMessage(data);
					if (msg.distance_robot_human_operator <= gantry_detection) {
						clearPercepts("human");
//						logger.info("I see the Gantry robot in less than 15 meters!");
						addPercept("human",Literal.parseLiteral("gantry_detected"));
					}
//					logger.info(msg.time+"");
//					logger.info(msg.human_operator_speed+"");
//					logger.info(msg.robot_speed+"");
//					logger.info(msg.distance_robot_human_operator+"");
				}
			}
	);
	}
	
		/* Subscriber for getting the information that the Gantry has been disabled. Note that the topic is made up, it should be replaced with the correct one later */ 
		/*
		bridge.subscribe(SubscriptionRequestMsg.generate("/gantry_disabled")
				.setType("std_msgs/Bool")
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {

				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<PrimitiveMsg<boolean>> unpacker = new MessageUnpacker<PrimitiveMsg<boolean>>(PrimitiveMsg.class);
					PrimitiveMsg<boolean> msg = unpacker.unpackRosMessage(data);
					if (msg.data) {
						clearPercepts("human");
//						logger.info("Gantry has been disabled!");
						addPercept("human",Literal.parseLiteral("gantry_disabled"));
					}
				}
			}
	);
	*/
	
		/* Subscriber for move_base result. Note that this will change in ROS2, needs experimentation */
		/*
		bridge.subscribe(SubscriptionRequestMsg.generate("/move_base/result")
				.setType("move_base_msgs/MoveBaseActionResult"),
//				.setThrottleRate(1)
//				.setQueueLength(1),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<MoveBaseActionResult> unpacker = new MessageUnpacker<MoveBaseActionResult>(MoveBaseActionResult.class);
					MoveBaseActionResult msg = unpacker.unpackRosMessage(data);
					clearPercepts();
//					System.out.println("Frame id: "+msg.header.frame_id);
//					System.out.println("Stamp sec: "+msg.header.stamp.secs);
//					System.out.println("Seq: "+msg.header.seq);
//					System.out.println("Goal: "+msg.status.goal_id.id);
//					System.out.println("Stamp sec: "+msg.status.goal_id.stamp.secs);
//					System.out.println("Status: "+msg.status.status);
//					System.out.println("Text: "+msg.status.text);
//					System.out.println();
					Literal movebase_result = new Literal("movebase_result");
					movebase_result.addTerm(new NumberTermImpl(msg.header.seq));
					movebase_result.addTerm(new NumberTermImpl(msg.status.status));
					addPercept(movebase_result);
				}
			}
	    );
		*/
    

    @Override
    public boolean executeAction(String agName, Structure action) {
		if (action.getFunctor().equals("move")) {
			//move((NumberTerm) action.getTerm(0).solve(),(NumberTerm) action.getTerm(1).solve(),(NumberTerm) action.getTerm(2).solve());
			move(0.0, 0.0, 0.0);
		}
		else if (action.getFunctor().equals("stop_movement")) { 
			stop_moving();
		}
		else if (action.getFunctor().equals("teleport_safe")) { 
			teleport();
		}
		else {
			logger.info("executing: "+action+", but not implemented!");
		}
        informAgsEnvironmentChanged();
        return true; // the action was executed with success
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
		move_base.publish(new Vector3(x,y,z));
	}
	
	// This method also needs to cancel any ongoing move base goals
	public void stop_moving() {
		Vector3 linear = new Vector3(0.0,0.0,0.0);
		Vector3 angular = new Vector3(0.0,0.0,0.0);
		cmd_vel.publish(new Twist(linear, angular));
	}
	
	// Should call a service by sending a message to a topic and having a Python script reading it to send the service request
	public void teleport() {
		
	}

    /** Called before the end of MAS execution */
    @Override
    public void stop() {
        super.stop();
    }
 }
