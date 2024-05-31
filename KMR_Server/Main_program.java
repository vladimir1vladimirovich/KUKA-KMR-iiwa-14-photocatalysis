package ItmoInfochemLab;

import java.util.HashSet;
import java.util.Set;
import javax.inject.Inject;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.kuka.common.ThreadUtil;
import com.kuka.nav.Pose;
import com.kuka.nav.fleet.ChangeGraphCommand;
import com.kuka.nav.fleet.FleetManager;
import com.kuka.nav.fleet.GraphMotion;
import com.kuka.nav.fleet.GraphMotionContainer;
import com.kuka.nav.fleet.RemoveFromGraphCommand;
import com.kuka.nav.fleet.actions.CustomNodeAction;
import com.kuka.nav.fleet.actions.CustomNodeActionContext;
import com.kuka.nav.fleet.actions.NodeActionExecutionType;
import com.kuka.nav.fleet.filter.InstanceFilter;
import com.kuka.nav.fleet.graph.GraphData;
import com.kuka.nav.fleet.graph.TopologyGraph;
import com.kuka.nav.fleet.graph.TopologyNode;
import com.kuka.nav.line.VirtualLineMotion;
import com.kuka.nav.rel.RelativeMotion;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.nav.robot.MobileRobotManager;
import com.kuka.nav.task.NavTaskCategory;
import com.kuka.nav.task.remote.RemoteTaskId;
import com.kuka.nav.task.remote.TaskRequest;
import com.kuka.nav.task.remote.TaskRequestContainer;
import com.kuka.resource.locking.LockException;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.task.ITaskLogger;
import com.kuka.task.RoboticsAPITask;


@NavTaskCategory
public class Main_program extends RoboticsAPITask {
	@Inject 
	private GraphData _graphData;
	@Inject
	private ITaskLogger _log;
	@Inject
	private MobileRobotManager _kmpMan; 
	@Inject
	private FleetManager _fleetMan;
	private static MobileRobot _kmp;
	@Inject
	private CustomActionTable1 _action1;
	@Inject
	private CustomActionTable2 _action2;
	@Inject
	private CustomActionTable1 _action3;
	@Inject
	private CustomActionTable2 _action4;
	private int _kmpId, graphId;
	private TopologyGraph tg;
	
	
	@Override
	public void initialize() {
		try {
			_kmpId =  1;
			_kmp = _kmpMan.getRobot(_kmpId);
			_fleetMan.addNodeCustomAction("action_1", _action1);
			_fleetMan.addNodeCustomAction("action_2", _action2);
			_fleetMan.addNodeCustomAction("action_3", _action3);
			_fleetMan.addNodeCustomAction("action_4", _action4);
			MQTT_Platform.initializeMQTT("Kuka_platform");
			MQTT_Platform.subscribe("state/coordsFromCamera");
			_kmp = _kmpMan.getRobot(1);
		} catch (Exception e) {
			_log.info(e.getMessage());
		}
	}
	
	
	@Override
	public void run() throws MqttException {
		int startNodeId = 1;
		graphId = 42;
		tg = _graphData.get(graphId);
	
		try {
			_kmp.lock();
			Pose startNodePose = new Pose(tg.getNode(startNodeId).getPosition().toPose(Math.toRadians(0)));
			VirtualLineMotion virtMo = new VirtualLineMotion(_kmp.getPose(), startNodePose);
			_kmp.execute(virtMo.setMaxVelocity(0.15).setMaxAcceleration(0.2));
			
			ChangeGraphCommand graCom1_2 = new ChangeGraphCommand(graphId, startNodeId);
			_kmp.execute(graCom1_2);
			_kmp.unlock();
					
			graphMotionToNode(4);
				
		} catch (Exception e) {
			e.printStackTrace();
			_log.info(e.getMessage());
		} finally {
			_kmp.unlock();
			MQTT_Platform.close();
		}
	}
	
	
	private void graphMotionToNode(int goalNode) {
		TopologyNode topologyEndNode = tg.getNode(goalNode);
		GraphMotion gm = new GraphMotion(tg, topologyEndNode);
		InstanceFilter filter = new InstanceFilter(_kmpId); 
	    gm.setResourceFilter(filter);
	    GraphMotionContainer graMoCon = _fleetMan.execute(gm);
	    graMoCon.awaitFinalized();
	}

	
}













