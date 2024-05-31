package photocatalysis;


import javax.inject.Inject;
import javax.inject.Named;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.TrioSignalsIOGroup;
import com.kuka.math.geometry.Transformation;
import com.kuka.nav.Pose;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.task.ITaskLogger;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.OrientationReferenceSystem;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.TimeUnit;
import java.net.*;
import java.io.*;
import java.lang.Math;


public class Table4_main extends RoboticsAPIApplication {
	@Inject
	private ITaskLogger _log;

	@Inject
	private LBR iiwa;
	
	Table4_actions table4 = new Table4_actions();
	
	@Named("Gripper1")
	@Inject
	Tool gripper;
	
	int samples = 20;
	
	
	@Override
	public void initialize() {
		_log.info("*** Table4_main program started ***");	
		table4.initialize(gripper, iiwa, _log, getApplicationControl());
	}
	
	
	@Override
	public void run() throws IOException, UnknownHostException, MqttException {
		try {

			IIWA.moveToDrivePos(gripper, iiwa, _log, getApplicationControl());
			table4.turnOffUvLed(_log, getApplicationControl());
			table4.closeUpperDoor(_log, getApplicationControl());
			table4.openUpperDoor(_log, getApplicationControl());
			ThreadUtil.milliSleep(7000);
			table4.detectMarkersAndInitializationFrames(new int[] {51, 52, 55, 54}, gripper, iiwa, _log, getApplicationControl());
			table4.moveToGeneralPoint(gripper, iiwa, _log, getApplicationControl());
			table4.closeUpperDoor(_log, getApplicationControl());
			ThreadUtil.milliSleep(7000);
			table4.closeFrontDoor(_log, getApplicationControl());
			table4.openFrontDoor(_log, getApplicationControl());
			ThreadUtil.milliSleep(7000);
			table4.actionWithGlass(gripper, iiwa, _log, getApplicationControl());
			long time1 = System.currentTimeMillis();
			table4.openSpectrophotometerDoor(gripper, iiwa, _log, getApplicationControl());
			table4.detectMarkersAndInitializationFrames(new int[] {53}, gripper, iiwa, _log, getApplicationControl());
			table4.moveToGeneralPoint(gripper, iiwa, _log, getApplicationControl());
			table4.moveCuvetteInSpectrophotometer(gripper, iiwa, _log, getApplicationControl());
			for (int sample_number=9; sample_number<=samples; sample_number++) {
				table4.moveToGeneralPoint(gripper, iiwa, _log, getApplicationControl());
				if (sample_number == 2) {
					table4.actionWithTare(gripper, iiwa, _log, getApplicationControl());
				}
				table4.putOnAdaptors(gripper, iiwa, _log, getApplicationControl());
				table4.takeDispenserMl(gripper, iiwa, _log, getApplicationControl());
				while ((System.currentTimeMillis()-table4.time1)/1000 <= 5*60) {
					_log.info("Waiting: " + String.valueOf((System.currentTimeMillis()-time1)/1000 - 5*60 + " sec"));
					ThreadUtil.milliSleep(1000);
				}
				table4.turnOffUvLed(_log, getApplicationControl());
				table4.openUpperDoor(_log, getApplicationControl());
				ThreadUtil.milliSleep(7000);
				table4.doseMl(gripper, iiwa, _log, getApplicationControl());
				table4.returnDispenserMl(gripper, iiwa, _log, getApplicationControl());
				table4.removeOnAdaptors(gripper, iiwa, _log, getApplicationControl());
				IIWA.moveToDrivePos(gripper, iiwa, _log, getApplicationControl());
				table4.closeSpectrophotometerDoor(gripper, iiwa, _log, getApplicationControl());
				table4.pressEnter(gripper, iiwa, _log, getApplicationControl());
				TimeUnit.SECONDS.sleep(60*3);
				table4.openSpectrophotometerDoor(gripper, iiwa, _log, getApplicationControl());
				table4.drainLiquidFromCuvette(sample_number, gripper, iiwa, _log, getApplicationControl());
			}
			
			table4.moveToGeneralPoint(gripper, iiwa, _log, getApplicationControl());
			table4.closeSpectrophotometerDoor(gripper, iiwa, _log, getApplicationControl());
			table4.turnOffUvLed(_log, getApplicationControl());
			table4.turnOffStirrer(_log, getApplicationControl());
			table4.openFrontDoor(_log, getApplicationControl());
			ThreadUtil.milliSleep(7000);
			table4.moveToGeneralPoint(gripper, iiwa, _log, getApplicationControl());
			table4.retaurnGlassToPlatform(gripper, iiwa, _log, getApplicationControl());
			table4.closeFrontDoor(_log, getApplicationControl());
			IIWA.moveToDrivePos(gripper, iiwa, _log, getApplicationControl());
					
        } catch (Exception e) {
			_log.info(e.getMessage());
			getApplicationControl().pause();
        } finally {
        	MQTT.close();
        	_log.info("*** Table4_main program finished ***");
        }
	}
	
}





