package photocatalysis;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.batch;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import java.io.IOException;
import java.net.UnknownHostException;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.task.ITaskLogger;



public class Table3_actions {

	private Frame frameAuxillaryBeforeLiquid1 = new Frame(867.8674402258655, 475.77283215243153, 805.293399873726, 0.0, 1.5706908941389166, -0.7652899478972068);
	private Frame frameAuxillaryBeforeLiquid2 = new Frame(870.5249025080968, 470.4522568259172, 805.3598848420113, 0.0, 1.5707323383677962, -0.15947496721635773);
	private JointPosition frameAuxillaryBeforeLiquid3 = new JointPosition(0.90998802310291, 0.8922524487746478, -1.2092884074765, -0.43321349165574335, 0.3685892180798063, 0.5496526403015298, 0.5169355266296234);	
	
	private Frame frameInNosesStand = new Frame (-112.5596889322932, -818.8397759048794, 311.5013264336105, 0.0, Math.PI/2, 0.8801157227696423); //250Z
	private Frame frameStandWithNosesUp2 = new Frame (-112.5596889322932, -818.8397759048794, 391.67380711943753, 0.0, Math.PI/2, 0.8801157227696423); //301
	private Frame frameStandWithNosesUp1 = new Frame (-112.5596889322932, -818.8397759048794, 734.2163500272519, 0.0, Math.PI/2, 0.8801157227696423);
	
	
	private Frame frameDispenserInStand = new Frame (-438.28682732981355, -694.893422458638, 275.50279313073, -2.096880175276322, Math.PI/2, 1.0398784307294253);
	private Frame frameDispenserStandUp = new Frame (-438.28682732981355, -694.893422458638, 618.9748476539523, -2.096880175276322, Math.PI/2, 1.0398784307294253); 
	private Frame frameDispenserInFrontOfStand = new Frame (-273.66283812730506, -694.893422458638, 275.50279313073, -2.096880175276322, Math.PI/2, 1.0398784307294253);
	
	private Frame frameDispenserInStandHigher = new Frame (-438.28682732981355, -694.893422458638, 341.185008407854, -2.096880175276322, Math.PI/2, 1.0398784307294253);
	private Frame frameDispenserInFrontOfStandHigher = new Frame (-273.66283812730506, -694.893422458638, 341.185008407854, -2.096880175276322, Math.PI/2, 1.0398784307294253);
	
	private Frame detect49Marker = new Frame(810.9682420662248, -323.95602720001455, 454.342916844636, 0.0, Math.PI/2, 0.0);
	private Frame detect47Marker = new Frame(503.7514890915755, -101.29850706326718, 352.5148048322144, Math.PI, 0.0, Math.PI);
	private Frame detect41Marker = new Frame(499.3311442490076, 103.2119785414074, 197.24978372517631, Math.PI, 0.0, Math.PI);
	private Frame detect45Marker = new Frame(532.7028569260975, 334.7685163177591, 234.8449781064265, Math.PI, 0.0, Math.PI);
	private Frame detect48Marker = new Frame(703.0267601968054, 270.3146584808989, 293.56829572621047, Math.PI, 0.0, Math.PI);
	
	private Frame auxillaryPointForDetect49Marker = new Frame(276.34467736356027, -512.0044501203281, 454.3144220396692, 0.0, Math.PI/2, 0.0);
	private Frame auxillaryPointForDetect47Marker = new Frame(533.7528895566116, -333.5194711044611, 452.7657888077438, Math.PI, 0.0, Math.PI);
	private Frame auxillaryPointForDetect41Marker = new Frame(499.3311442490076, 103.2119785414074, 352.5148048322144, Math.PI, 0.0, Math.PI);
	private Frame auxillaryPointForDetect45Marker = new Frame(417.0063526028766, 176.7261210335386, 335.41313846207373, Math.PI, 0.0, Math.PI);
	private Frame auxillaryPointForDetect48Marker = new Frame(532.7028569260975, 334.7685163177591, 293.56829572621047, Math.PI, 0.0, Math.PI);
	
	private Frame generalPointAboveTable = new Frame(630.6625734841773, -0.7415529209594043, 454.8133001493577, Math.PI, 0.0, Math.PI);
	private Frame generalPointAboveKuka = new Frame(-1.6878215547209598, -560.1670393802813, 454.8166147343991, Math.PI/2, 0.0, Math.PI);
	
	private Frame auxillaryPointForDispenserInFrontOfVolumeChanger1 = new Frame(-438.3807076220236, -491.83832535237667, 890.5147527419973, 0.0, 1.5704790212568702, 3.1369169556198764);
	private Frame auxillaryPointForDispenserInFrontOfVolumeChanger2 = new Frame(135.00784764351735, -505.15343211845556, 756.6308690100607, 0.0, 1.5706436709880238, 0.06333887789664158);
	
	private Frame glassStandPlatformUp1 = new Frame(-435.50958369416674, -512.7771705302214, 400.0, 2.5419661263150544, 1.5688763111658681, -2.8511667296342336);
	private Frame glassStandPlatformUp2 = new Frame(-435.50958369416674, -512.7771705302214, 235.163733777917, 2.5419661263150544, 1.5688763111658681, -2.8511667296342336);
	private Frame glassStandPlatformDown = new Frame(-435.4357208281991, -512.930024551879, 56.86541733511934, 2.6374171247991636, 1.5685539048886752, -2.755837339482935);
	private JointPosition frameAuxillaryGlassStandPlatform1 = new JointPosition(-0.03385861114442209, -0.9111255656593408, -1.209325558554958, -1.6486466906289645, 0.6964159646504026, 1.5164137117885235, -1.3376892506541413);
	private JointPosition frameAuxillaryGlassStandPlatform2 = new JointPosition(0.18442859266514117, -0.7792927400927504, -2.9143140453630445, -1.2783046000059535, 1.3519340839461582, 1.6538900287300093, -2.0396330469169524);
	
	private Frame marker49_frame;
	private Frame marker47_frame;
	private Frame marker41_frame;
	private Frame marker45_frame;
	private Frame marker48_frame;
	
	long time;
		
	
	public void initialize(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			_log.info("Table3 initialization");
			MQTT.initializeMQTT("Table3");
			MQTT.subscribe("state/transformCoords");
			MQTT.subscribe("state/gripper/rotate");
			MQTT.subscribe("state/clamp/compress");
			MQTT.subscribe("state/clamp/rotate_clockwise");
			MQTT.subscribe("state/clamp/rotate_counterclockwise");
			MQTT.subscribe("state/volume_changing_device/compress");
			MQTT.subscribe("state/volume_changing_device/rotate_clockwise");
			MQTT.subscribe("state/volume_changing_device/rotate_counterclockwise");
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
		gripper.attachTo(iiwa.getFlange());
	}
		
	
	public void detectMarkersAndInitializationFrames(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("/TCP1").move(batch(
					ptp(auxillaryPointForDetect49Marker).setJointVelocityRel(1.0).setBlendingCart(200),
					lin(detect49Marker).setJointVelocityRel(1.0)
					));
			ThreadUtil.milliSleep(3000);
			marker49_frame = MQTT.getMarkerFrame(49, 30, iiwa);
								
			clampByDevice(105, _log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
					lin(auxillaryPointForDetect47Marker).setJointVelocityRel(1.0),
					lin(detect47Marker).setJointVelocityRel(1.0)
					));
			ThreadUtil.milliSleep(3000);
			marker47_frame = MQTT.getMarkerFrame(47, 30, iiwa);
			
			gripper.getFrame("/TCP1").move(batch(
					ptp(auxillaryPointForDetect41Marker).setJointVelocityRel(0.8).setBlendingCart(50),
					lin(detect41Marker).setJointVelocityRel(0.8)
					));
			ThreadUtil.milliSleep(3000);
			marker41_frame = MQTT.getMarkerFrame(41, 29, iiwa);
			
			gripper.getFrame("/TCP1").move(batch(
					ptp(auxillaryPointForDetect45Marker).setJointVelocityRel(0.8).setBlendingCart(100),
					lin(detect45Marker).setJointVelocityRel(0.8)
					));
			ThreadUtil.milliSleep(3000);
			marker45_frame = MQTT.getMarkerFrame(45, 29, iiwa);
			
			gripper.getFrame("/TCP1").move(batch(
					lin(auxillaryPointForDetect48Marker).setJointVelocityRel(1.0).setBlendingCart(30),
					lin(detect48Marker).setJointVelocityRel(1.0)
					));
			ThreadUtil.milliSleep(3000);
			marker48_frame = MQTT.getMarkerFrame(48, 30, iiwa);
				
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void changeDispenserVolume (Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MyGripper.gripperAction(180, _log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
					lin(generalPointAboveTable).setCartVelocity(800).setBlendingCart(100),
					lin(generalPointAboveKuka).setJointVelocityRel(1.0).setBlendingCart(200),
					ptp(frameDispenserInFrontOfStand).setJointVelocityRel(1.0).setBlendingCart(100),
					lin(frameDispenserInStand).setJointVelocityRel(0.4)
					));
			MyGripper.gripperAction(105, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(frameDispenserStandUp).setJointVelocityRel(0.4).setBlendingCart(200),
					lin(auxillaryPointForDispenserInFrontOfVolumeChanger1).setJointVelocityRel(1.0).setBlendingCart(100),
					lin(auxillaryPointForDispenserInFrontOfVolumeChanger2).setJointVelocityRel(1.0).setBlendingCart(100),
					ptp(auxillaryPointForDetect49Marker).setJointVelocityRel(1.0).setBlendingCart(200),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker49_frame, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0)).setCartVelocity(200).setBlendingCart(700),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker49_frame, 0.0, -264.0, 100.0, 0.0, 0.0, 0.0)).setCartVelocity(100).setBlendingCart(500)
					));
			gripper.getFrame("/TCP2").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker49_frame, 0.0, -264.0, -25.0, 0.0, 0.0, 0.0)).setCartVelocity(100).setBlendingCart(100));
			clampDispenserCap(50, _log, getAplControl);//разжатие
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP2").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker49_frame, 0.0, -230.5, -25.0, 0.0, 0.0, 0.0)).setCartVelocity(100));
			clampDispenserCap(115, _log, getAplControl);//сжатие
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP2").moveAsync(lin(CoordinatesTransformation.transformCoordsToFlange(marker49_frame, 0.0, -228.5, -25.0, 0.0, 0.0, 0.0)).setCartVelocity(0.2));
			decreaseDispenserVolume(1.5, _log, getAplControl);
			clampDispenserCap(50, _log, getAplControl);//разжатие
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP2").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker49_frame, 0.0, -264.0, -25.0, 0.0, 0.0, 0.0)).setCartVelocity(100).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker49_frame, 0.0, -264.0, 100.0, 0.0, 0.0, 0.0)).setCartVelocity(100).setBlendingCart(50)
					));
			gripper.getFrame("/TCP1").move(batch(
					lin(auxillaryPointForDetect49Marker).setJointVelocityRel(1.0).setBlendingCart(100),
					lin(auxillaryPointForDispenserInFrontOfVolumeChanger2).setJointVelocityRel(1.0).setBlendingCart(100)
					));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void putDispencerNose (Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("/TCP1").move(batch(
					lin(auxillaryPointForDispenserInFrontOfVolumeChanger1).setJointVelocityRel(1.0).setBlendingCart(100),
					lin(frameDispenserStandUp).setJointVelocityRel(1.0).setBlendingCart(100),
					lin(frameDispenserInStand).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(frameDispenserInFrontOfStand).setJointVelocityRel(0.5).setBlendingCart(50),
					lin(frameDispenserInFrontOfStandHigher).setJointVelocityRel(0.5).setBlendingCart(50),
					lin(frameDispenserInStandHigher).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(105, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(frameDispenserStandUp).setJointVelocityRel(0.4).setBlendingCart(100),
					lin(auxillaryPointForDispenserInFrontOfVolumeChanger1).setJointVelocityRel(1.0).setBlendingCart(100),
					lin(auxillaryPointForDispenserInFrontOfVolumeChanger2).setJointVelocityRel(1.0).setBlendingCart(100),
					lin(frameStandWithNosesUp1).setJointVelocityRel(1.0).setBlendingCart(300),
					lin(frameStandWithNosesUp2).setJointVelocityRel(0.5),
					lin(frameInNosesStand).setJointVelocityRel(0.02),
					lin(frameStandWithNosesUp2).setJointVelocityRel(0.1),
					lin(frameStandWithNosesUp1).setJointVelocityRel(0.5).setBlendingCart(300),
					lin(auxillaryPointForDispenserInFrontOfVolumeChanger1).setJointVelocityRel(1.0).setBlendingCart(100),
					lin(frameDispenserStandUp).setJointVelocityRel(1.0).setBlendingCart(10),
					lin(frameDispenserInStandHigher).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(frameDispenserInFrontOfStandHigher).setJointVelocityRel(0.4).setBlendingCart(10),
					lin(generalPointAboveKuka).setJointVelocityRel(1.0).setBlendingCart(300)
					));
			
		} catch (Exception e) {
				_log.info(e.getMessage());
				getAplControl.pause();
				_log.info("Program paused");
		}
	}
	
	
	public void openTube(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			
			MyGripper.gripperAction(180, _log, getAplControl);
			gripper.getFrame("/TCP1").move(lin(generalPointAboveTable).setCartVelocity(800).setBlendingCart(50));
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker41_frame, -42.0, -41.0, 200.0, 0.0, 0.0, 0.0)).setCartVelocity(150).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker41_frame, -42.0, -41.0, 120.0, 0.0, 0.0, 0.0)).setCartVelocity(150)
					));
			MyGripper.gripperAction(105, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker41_frame, -42.0, -41.0, 350.0, 0.0, 0.0, 0.0)).setCartVelocity(150).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker47_frame, 0.0, 0.0, 175.0, 0.0, 0.0, 0.0)).setCartVelocity(100).setBlendingCart(30),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker47_frame, 0.0, 0.0, 115.0, 0.0, 0.0, 0.0)).setCartVelocity(20)
					));
			clampByDevice(70, _log, getAplControl);
			ThreadUtil.milliSleep(2000);
			openByDevice(3.0, _log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker47_frame, 0.0, 0.0, 141.0, 0.0, 0.0, 0.0)).setCartVelocity(3),
					lin(generalPointAboveTable).setCartVelocity(100),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker48_frame, 150.0, 0.0, 110.0, 0.0, 0.0, Math.toRadians(60))).setCartVelocity(150)
					));
			
		} catch (Exception e) {
				_log.info(e.getMessage());
				getAplControl.pause();
				_log.info("Program paused");
		  }
	}
	
	
	public void poorLiquid (Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP2").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker47_frame, 0.0, 0.0, 220.0, Math.toRadians(-45), Math.toRadians(90), 0.0)).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker47_frame, 0.0, 0.0, 220.0, Math.toRadians(45), 0.0, Math.toRadians(90))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker47_frame, 0.0, 0.0, 90.0, Math.toRadians(45), 0.0, Math.toRadians(90))).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(93, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			clampByDevice(105, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP2").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker47_frame, 0.0, 0.0, 200.0, Math.toRadians(45), 0.0, Math.toRadians(90))).setJointVelocityRel(0.2).setBlendingCart(30),
					lin(frameAuxillaryBeforeLiquid1).setJointVelocityRel(0.2).setBlendingCart(300),
					lin(frameAuxillaryBeforeLiquid2).setJointVelocityRel(1.0),
					ptp(frameAuxillaryBeforeLiquid3).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -20.0, -90.0, 250.0, Math.toRadians(-30), 0.0, Math.toRadians(90))).setJointVelocityRel(0.2).setBlendingCart(20),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -8.0, -107.0, 165.0, Math.toRadians(-30), 0.0, Math.toRadians(90))).setJointVelocityRel(0.2),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -8.0, -107.0, 100.0, Math.toRadians(45), Math.toRadians(-90), Math.toRadians(-30))).setJointVelocityRel(0.1)
					));
			ThreadUtil.milliSleep(2000);
			gripper.getFrame("/TCP2").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -8.0, -107.0, 190.0, Math.toRadians(45), Math.toRadians(-90), Math.toRadians(-40))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -8.0, -107.0, 100.0, Math.toRadians(45), Math.toRadians(-90), Math.toRadians(-40))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -8.0, -107.0, 190.0, Math.toRadians(45), Math.toRadians(-90), Math.toRadians(-40))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -20.0, -90.0, 165.0, Math.toRadians(-30), 0.0, Math.toRadians(90))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -20.0, -90.0, 250.0, Math.toRadians(-30), 0.0, Math.toRadians(90))).setJointVelocityRel(1.0).setBlendingCart(20),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker48_frame, 150.0, 0.0, 140.0, 0.0, 0.0, Math.toRadians(60))).setJointVelocityRel(1.0).setBlendingCart(30),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker48_frame, 150.0, 0.0, 70.0, 0.0, 0.0, Math.toRadians(60))).setJointVelocityRel(1.0)
					));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP2").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker48_frame, 150.0, 0.0, 140.0, 0.0, 0.0, Math.toRadians(60))).setJointVelocityRel(1.0).setBlendingCart(30));
			
		} catch (Exception e) {
				_log.info(e.getMessage());
				getAplControl.pause();
				_log.info("Program paused");
		}
	}
	
	
	public void putCapOnBeaker(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("/TCP1").move(batch(
					lin(auxillaryPointForDetect45Marker).setJointVelocityRel(0.5).setBlendingCart(30),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, 45.0, -78.0, 100.0, 0.0, 0.0, 0.0)).setJointVelocityRel(1.0).setBlendingCart(30),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, 45.0, -78.0, 30.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(95, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, 45.0, -78.0, 100.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1).setBlendingCart(5),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -45.0, -78.0, 100.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1).setBlendingCart(5),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -45.0, -78.0, 76.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -45.0, -78.0, 150.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1));
			
		} catch (Exception e) {
				_log.info(e.getMessage());
				getAplControl.pause();
				_log.info("Program paused");
		}
	}
	
	
	public void moveBeakerToPlatform(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("/TCP2").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -45.0, -78.0, 150.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.3));
			gripper.getFrame("/TCP3").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -45.0, -78.0, 130.0, Math.toRadians(-30), 0.0, Math.toRadians(90))).setJointVelocityRel(0.3));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP3").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -45.0, -78.0, 34.0, Math.toRadians(-30), 0.0, Math.toRadians(90))).setJointVelocityRel(0.1));
			MyGripper.gripperAction(95, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP3").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker45_frame, -45.0, -78.0, 230.0, Math.toRadians(-30), 0.0, Math.toRadians(90))).setJointVelocityRel(0.3).setBlendingCart(50));
			gripper.getFrame("/TCP2").move(batch(
					lin(frameAuxillaryBeforeLiquid2).setJointVelocityRel(0.3).setBlendingCart(300),
					lin(frameAuxillaryBeforeLiquid1).setJointVelocityRel(0.3).setBlendingCart(300)
					));
			gripper.getFrame("/TCP3").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker47_frame, 0.0, 0.0, 510.0, Math.toRadians(45), 0.0, Math.toRadians(90))).setJointVelocityRel(0.3).setBlendingCart(300));
			gripper.getFrame("/TCP1").move(batch(
					ptp(frameAuxillaryGlassStandPlatform1).setJointVelocityRel(0.3).setBlendingCart(100),
					ptp(frameAuxillaryGlassStandPlatform2).setJointVelocityRel(0.3).setBlendingCart(100),
					ptp(glassStandPlatformUp1).setJointVelocityRel(0.3).setBlendingCart(100),
					lin(glassStandPlatformUp2).setJointVelocityRel(0.3),
					lin(glassStandPlatformDown).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(lin(glassStandPlatformUp1).setJointVelocityRel(0.3).setBlendingCart(100));
			
		} catch (Exception e) {
				_log.info(e.getMessage());
				getAplControl.pause();
				_log.info("Program paused");
		}
	}
		
	
	public void clampByDevice(int degrees, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/clamp/compress", String.valueOf(degrees));
			time = System.currentTimeMillis();
			while (MQTT.data_clamp_compress != degrees) {
				if ((System.currentTimeMillis()-time)/1000 >= 4) {
					_log.info("Unsuccessful compress clamping device into " + String.valueOf(degrees));
					_log.info("Try to compress clamping device into " + String.valueOf(degrees) + " ...");
					MQTT.publish("data/clamp/compress", String.valueOf(degrees));
					time = System.currentTimeMillis();
				}
				ThreadUtil.milliSleep(50);
			}
			_log.info("Successful compress clamping device into " + String.valueOf(degrees));
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
		 	_log.info("Program paused");
	    }
	}
	
	
	public void openByDevice(double revolutions, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.data_clamp_rotate_clockwise = 0.0;
			MQTT.stateClampStr = "";
			MQTT.publish("data/clamp/rotate_clockwise", String.valueOf(revolutions));
			time = System.currentTimeMillis();
			while (!MQTT.stateClampStr.contains("Started rotate clockwise ") & MQTT.data_clamp_rotate_clockwise != revolutions) {
				if ((System.currentTimeMillis()-time)/1000 >= 3*revolutions+3) {
					_log.info("Unsuccessful opening by clamping device " + String.valueOf(revolutions) + " revolutions");
					_log.info("Try to opening by clamping device " + String.valueOf(revolutions) + " revolutions ...");
					MQTT.publish("data/clamp/rotate_clockwise", String.valueOf(revolutions));
					time = System.currentTimeMillis();
				}
				ThreadUtil.milliSleep(50);
			}
			_log.info("Successful opening by clamping device " + String.valueOf(revolutions) + " revolutions");
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
		 	_log.info("Program paused");
	    }
	}
	  
	 
	public void closeByDevice(double revolutions, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.data_clamp_rotate_counterclockwise = 0.0;
			MQTT.stateClampStr = "";
			MQTT.publish("data/clamp/rotate_counterclockwise", String.valueOf(revolutions));
			time = System.currentTimeMillis();
			while (!MQTT.stateClampStr.contains("Finished rotate counterclockwise ") & MQTT.data_clamp_rotate_counterclockwise != revolutions) {
				if ((System.currentTimeMillis()-time)/1000 >= 3*revolutions+3) {
					_log.info("Unsuccessful closing by clamping device " + String.valueOf(revolutions) + " revolutions");
					_log.info("Try to closing by clamping device " + String.valueOf(revolutions) + " revolutions ...");
					MQTT.publish("data/clamp/rotate_counterclockwise", String.valueOf(revolutions));
					time = System.currentTimeMillis();
				}
				ThreadUtil.milliSleep(50);
			}
			_log.info("Successful closing by clamping device " + String.valueOf(revolutions) + " revolutions");
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
		 	_log.info("Program paused");
	    }
	}

	
	public void clampDispenserCap(int degrees, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/volume_changing_device/compress", String.valueOf(degrees));
			time = System.currentTimeMillis();
			while (MQTT.data_volume_changer_compress != degrees) {
				if ((System.currentTimeMillis()-time)/1000 >= 4) {
					_log.info("Unsuccessful compress volume changing device into " + String.valueOf(degrees));
					_log.info("Try to compress volume changing device into " + String.valueOf(degrees) + " ...");
					MQTT.publish("data/volume_changing_device/compress", String.valueOf(degrees));
					time = System.currentTimeMillis();
				}
				ThreadUtil.milliSleep(50);
			}
			_log.info("Successful compress volume changing device into " + String.valueOf(degrees));
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void increaseDispenserVolume(double revolutions, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.data_volume_changer_rotate_clockwise = 0.0;
			MQTT.stateVolumeChangerStr = "";
			MQTT.publish("data/volume_changing_device/rotate_clockwise", String.valueOf(revolutions));
			time = System.currentTimeMillis();
			while (!MQTT.stateVolumeChangerStr.contains("Finished rotate clockwise ") & MQTT.data_volume_changer_rotate_clockwise != revolutions) {
				if ((System.currentTimeMillis()-time)/1000 >= 6.5*revolutions+5) {
					_log.info("Unsuccessful increasing volume by " + String.valueOf(revolutions) + " revolutions");
					_log.info("Try to increasing volume by " + String.valueOf(revolutions) + " revolutions ...");
					MQTT.publish("data/volume_changing_device/rotate_clockwise", String.valueOf(revolutions));
					time = System.currentTimeMillis();
				}
				ThreadUtil.milliSleep(50);
			}
			_log.info("Successful increasing volume by " + String.valueOf(revolutions) + " revolutions");
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}

	
	public void decreaseDispenserVolume(double revolutions, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.data_volume_changer_rotate_counterclockwise = 0.0;
			MQTT.stateVolumeChangerStr = "";
			MQTT.publish("data/volume_changing_device/rotate_counterclockwise", String.valueOf(revolutions));
			time = System.currentTimeMillis();
			while (!MQTT.stateVolumeChangerStr.contains("Finished rotate counterclockwise ") & MQTT.data_volume_changer_rotate_counterclockwise != revolutions) {
				if ((System.currentTimeMillis()-time)/1000 >= 6.5*revolutions+5) {
					_log.info("Unsuccessful decreasing volume by " + String.valueOf(revolutions) + " revolutions");
					_log.info("Try to decreasing volume by " + String.valueOf(revolutions) + " revolutions ...");
					MQTT.publish("data/volume_changing_device/rotate_counterclockwise", String.valueOf(revolutions));
					time = System.currentTimeMillis();
				}
				ThreadUtil.milliSleep(50);
			}
			_log.info("Successful decreasing volume by " + String.valueOf(revolutions) + " revolutions");
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
  
}
