package ItmoInfochemLab;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import javax.inject.Inject;
import javax.inject.Named;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.kuka.common.ThreadUtil;
import com.kuka.nav.fleet.ChangeGraphCommand;
import com.kuka.nav.fleet.actions.CustomNodeAction;
import com.kuka.nav.fleet.actions.CustomNodeActionContext;
import com.kuka.nav.fleet.actions.NodeActionExecutionType;
import com.kuka.nav.fleet.graph.TopologyNode;
import com.kuka.nav.fleet.graph.properties.CustomUserData;
import com.kuka.nav.fleet.graph.properties.NodeAction;
import com.kuka.nav.fleet.graph.properties.Property;
import com.kuka.nav.fleet.graph.properties.PropertyHolder;
import com.kuka.nav.rel.RelativeMotion;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.nav.task.remote.RemoteTaskId;
import com.kuka.nav.task.remote.TaskRequest;
import com.kuka.nav.task.remote.TaskRequestContainer;
import com.kuka.resource.locking.LockException;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.task.ITaskLogger;


public class CustomActionTable1 implements CustomNodeAction {
	@Inject
	private ITaskLogger _log;
	
	@Inject
	private MobileRobot kmp;

	
	@Override
	public void cancel() {
	}

	
	@Override
	public CustomNodeAction copy() {
		return this;
	}
	

	@Override
	public Set<NodeActionExecutionType> defineWhen(CustomNodeActionContext ctx) {
		Set<NodeActionExecutionType> set = new HashSet<NodeActionExecutionType>();
		set.add(NodeActionExecutionType.REACH);
		return set;
	}
	

	@Override
	public double getEstimatedExecutionTime(TopologyNode node) {
		return 10;
	}
	

	@Override
	public void run(CustomNodeActionContext ctx) {
		try {
			
			_log.info("Custom action table 1");
			int node_id = ctx.getNode().getId();
			int marker_id = 10;
			int marker_size = 48;
			int desired_X = 370; //Desired distance from the center of the platform to the center of the marker (by Xm)
			int desired_Z = 370; //Desired distance from the edge of the platform to the table (according to Zm)
			double[] platformCoords = new double[3];
			
			// Moving the manipulator into position to detect the table marker
			_log.info("Movement to camera_detection_tables for Table1");
			RemoteTaskId taskId1 = new RemoteTaskId("photocatalysis.MovementToDetectionPosition");
			TaskRequestContainer taskContainer1 = kmp.execute(new TaskRequest(taskId1));
			taskContainer1.awaitFinished();
			
			// Positioning the platform according to the table marker
			_log.info("Start platform positioning for Table1");
			platformCoords = MQTT_Platform.getTableMarkerFrame(marker_id, marker_size, desired_X, desired_Z);
			kmp.lock();
			kmp.execute(new RelativeMotion(platformCoords[0], platformCoords[1], platformCoords[2]).setMaxVelocity(0.1).setGoalAccuracyTrans(0.01)); //.setGoalAccuracyRot(0.5)
			kmp.unlock();
			
			// Executing a program (action) in a node
			_log.info("Start photocatalysis.Table1_main");
			RemoteTaskId taskId2 = new RemoteTaskId("photocatalysis.Table1_main");
			TaskRequestContainer taskContainer2 = kmp.execute(new TaskRequest(taskId2));
			taskContainer2.awaitFinished();

		} catch (Exception e) {
			e.printStackTrace();
			_log.info(e.getMessage());
	    }
	}
}

