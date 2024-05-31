package photocatalysis;

import javax.inject.Inject;
import javax.inject.Named;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.TrioSignalsIOGroup;
import com.kuka.math.geometry.Transformation;
import com.kuka.nav.Pose;
import com.kuka.nav.robot.MobileRobot;
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


public class tableKuka_main extends RoboticsAPIApplication {
	@Inject
	private ITaskLogger _log;

	@Inject
	private LBR iiwa;
	
	tableKuka_actions tableKuka = new tableKuka_actions();
	
	@Named("Gripper1")
	@Inject
	Tool gripper;
	
	
	@Override
	public void initialize() {
		_log.info("*** TableKuka_main program started ***");	
		tableKuka.initialize(gripper, iiwa, _log);
	}
	
	
	@Override
	public void run() throws IOException, UnknownHostException, MqttException {
		try {
			
			IIWA.moveToDrivePos(gripper, iiwa, _log, getApplicationControl());
			tableKuka.puttingOnAdaptors(gripper, iiwa, _log, getApplicationControl());
			tableKuka.dosingMl(gripper, iiwa, _log, getApplicationControl());
			tableKuka.droppingMl(gripper, iiwa, _log, getApplicationControl());
			tableKuka.dosingCuvette(gripper, iiwa, _log, getApplicationControl());
			
        } catch (Exception e) {
			_log.info(e.getMessage());
			getApplicationControl().pause();
        } finally {
        	MQTT.close();
        	_log.info("*** TableKuka_main program finished ***");
        }
	}
	
}





