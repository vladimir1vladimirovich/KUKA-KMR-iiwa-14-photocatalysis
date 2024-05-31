package photocatalysis;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.batch;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.task.ITaskLogger;


public class Table1_actions {
	
	private Frame detectMarkerFrame = new Frame(532.6780738722489, 20.741946869511498, 302.6234388294889, Math.PI, 0, Math.PI);
	private Frame intermediateFrame1 = new Frame(10.857989716732284, -340.1612543631679, 422.62458487735205, Math.PI/2, 0.0, -Math.PI);
	private Frame takeTareFrame = new Frame(89.18946388523378, -568.8996756714806, 53.45265529502882, Math.PI, 0, Math.PI);
	private Frame aboveTareFrame = new Frame(87.18946388523378, -568.8996756714806, 390.45265529502882, Math.PI, 0, Math.PI);
	private Frame aboveTareFrame1 = new Frame(87.23429802256884, -276.5656801804877, 396.87611786564304, Math.PI, 0, Math.PI);
	private Frame marker59_frame;
	private ForceSensorData sensorData;
	

	public void initialize(Tool gripper, LBR iiwa, ITaskLogger _log) {
		try {
			_log.info("Table1 initialization");
			MQTT.initializeMQTT("Table1");
			MQTT.subscribe("state/transformCoords");
			MQTT.subscribe("state/gripper/rotate");
		} catch (Exception e) {
			_log.info(e.getMessage());
		}
		gripper.attachTo(iiwa.getFlange());
	}
	
	
	public void detectMarkersAndInitializationFrames(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("/TCP1").move(batch(
					ptp(intermediateFrame1).setJointVelocityRel(1.0).setBlendingCart(200),
					lin(aboveTareFrame1).setJointVelocityRel(1.0).setBlendingCart(300),
					ptp(detectMarkerFrame).setJointVelocityRel(1.0)
					));
			ThreadUtil.milliSleep(4000);
			marker59_frame = MQTT.getMarkerFrame(59, 30, iiwa);
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void pickUpTare(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			
			gripper.getFrame("/TCP1").move(ptp(CoordinatesTransformation.transformCoordsToFlange(marker59_frame, -40.0, -60.0, 80.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1).setBlendingCart(30));
			MyGripper.gripperAction(100, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker59_frame, -40.0, -60.0, 25.0, 0.0, 0.0, 0.0)).setCartVelocity(30));
			MyGripper.gripperAction(38, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker59_frame, -40.0, -60.0, 80.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.3).setBlendingCart(30),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker59_frame, 30.0, -60.0, 80.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.3).setBlendingCart(30),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker59_frame, 30.0, -60.0, 21.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.3)
					));
			MyGripper.gripperAction(100, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker59_frame, 30.0, -60.0, 80.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.3).setBlendingCart(30),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker59_frame, -40.0, -67.0, 80.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.3).setBlendingCart(30),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker59_frame, -40.0, -67.0, 7.0, 0.0, 0.0, 0.0)).setCartVelocity(30)
					));
			MyGripper.gripperAction(75, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker59_frame, -40.0, -67.0, 80.0, 0.0, 0.0, 0.0)).setCartVelocity(50),
					lin(aboveTareFrame1).setCartVelocity(50).setBlendingCart(300),
					lin(aboveTareFrame).setCartVelocity(50).setBlendingCart(300),
					lin(takeTareFrame).setCartVelocity(50)
					));
			
			MyGripper.gripperAction(120, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(lin(aboveTareFrame).setCartVelocity(300).setBlendingCart(300));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
}
