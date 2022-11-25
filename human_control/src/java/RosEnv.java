import jason.asSyntax.*;
import jason.environment.*;
import java.util.logging.*;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.msgs.ariac_msgs.Snapshot;
import ros.tools.MessageUnpacker;
import com.fasterxml.jackson.databind.JsonNode;

public class RosEnv extends Environment {

    private Logger logger = Logger.getLogger("ariac_env."+RosEnv.class.getName());
    
    final int gantry_detection = 15;
    
    RosBridge bridge = new RosBridge();

    /** Called before the MAS execution with the args informed in .mas2j */
    @Override
    public void init(String[] args) {
        super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");
		
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
//						logger.info("I see the robot in less than 15 meters!");
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

    @Override
    public boolean executeAction(String agName, Structure action) {
		if (action.getFunctor().equals("hello_ros")) {
			hello_ros();
		}
		else {
			logger.info("executing: "+action+", but not implemented!");
		}
        informAgsEnvironmentChanged();
        return true; // the action was executed with success
    }
    
	public void hello_ros() {
		Publisher pub = new Publisher("/java_to_ros", "std_msgs/String", bridge);
		
		for(int i = 0; i < 100; i++) {
			pub.publish(new PrimitiveMsg<String>("hello from Jason " + i));
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

    /** Called before the end of MAS execution */
    @Override
    public void stop() {
        super.stop();
    }
}
