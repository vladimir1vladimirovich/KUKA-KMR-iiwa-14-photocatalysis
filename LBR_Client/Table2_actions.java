package photocatalysis;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.batch;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
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
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.task.ITaskLogger;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;

import java.net.*;
import java.io.*;
import java.lang.Math;


public class Table2_actions {

	private Frame frameForDetectionMarker22 = new Frame(732.8482917196787, -141.18216003404922, 666.2763754103316, -1.581012909971656, 1.5616846724309816, -1.5810478347432326);
	private Frame frameForDetectionMarker21Up = new Frame(639.1429620858266, 35.21300981949563, 958.0747260714799, Math.toRadians(130.75), Math.toRadians(89.57), Math.toRadians(130.74));
	private Frame frameForDetectionMarker21Down = new Frame (639.1591029158701, 35.21952591240693, 676.9711470923847, Math.toRadians(130.75), Math.toRadians(89.57), Math.toRadians(130.74));
	private Frame takeTare = new Frame (90.49682874037673, -568.0299938861223, 59.20442316880077, 3.016816050398784, 1.276023614073105, 3.013896860742084);
	private Frame aboveTare = new Frame (90.5979625993402, -567.9260225541044, 139.13344203802754, 3.016816050398784, 1.276023614073105, 3.013896860742084);
	private Frame intermediatePoint1 = new Frame (90.57704581729799, -567.8727575472133, 710.1542258386887, 3.0165460665486723, 1.276078114711663, 3.013315024845671);
	private Frame intermediatePoint2 = new Frame (536.8531689795954, -578.2233593768453, 677.1157149867062, 3.0168882868113753, 1.275583699073694, 3.0139799873902025);
	private Frame inFrontOfMuffle = new Frame(635.9482591661015, 107.6459973974013, 540.8036875061402, 3.014996860908419, 1.2822022832595834, 3.0096665523338992);
	private Frame marker22_frame;
	private Frame marker21_frame;
	private ForceSensorData sensorData;
	
	
	public void initialize(Tool gripper, LBR iiwa, ITaskLogger _log, String tableName, String fumeCupboardDoorState) {
		try {
			_log.info("Table2 initialization");
			MQTT.initializeMQTT(tableName);
			MQTT.subscribe("state/fumeCupboardDoor");
			MQTT.subscribe("state/transformCoords");
			MQTT.subscribe("state/gripper/rotate");
			MQTT.subscribe("state/muffle");
			MQTT.subscribe("state/muffle_relay");
			ThreadUtil.milliSleep(100);
			MQTT.publish("state/fumeCupboardDoor", fumeCupboardDoorState);
		} catch (Exception e) {
			_log.info(e.getMessage());
		}
		gripper.attachTo(iiwa.getFlange());
	}
	
	
	public void moveToGeneralPoint(Tool gripper, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			_log.info("Moving to general point of Table2");
			gripper.getFrame("/TCP1").move(ptp(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, 0.0, 0.0, 270.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.6));
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void detectMarkersAndInitializationFrames(Tool gripper, LBR iiwa, ITaskLogger _log, String marker_name, IApplicationControl getAplControl) {
		try {
			if (marker_name.equals("fume_cupboard")) {
				if (MQTT.stateFumeCupboardDoor.equals("1")) { 
					gripper.getFrame("/TCP1").move(ptp(frameForDetectionMarker21Up).setJointVelocityRel(0.5));
				}
				if (MQTT.stateFumeCupboardDoor.equals("0")) {
					gripper.getFrame("/TCP1").move(ptp(frameForDetectionMarker21Down).setJointVelocityRel(0.5));
				}
				ThreadUtil.milliSleep(3000);
				marker21_frame = MQTT.getMarkerFrame(21, 29, iiwa);
			}	
			if (marker_name.equals("muffle")) {	
				gripper.getFrame("/TCP1").move(ptp(frameForDetectionMarker22).setJointVelocityRel(0.5));
				ThreadUtil.milliSleep(3000);
				marker22_frame = MQTT.getMarkerFrame(22, 29, iiwa);
			}
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}

	
	public void muffleLockAction(String stateLock, ITaskLogger _log) {
	    try {
	    	MQTT.publish("data/muffle", stateLock);
	    	ThreadUtil.milliSleep(400);    
	    	if (stateLock.equals(MQTT.stateLockStr)) {
	    		_log.info("Successful turn the muffle lock to " + String.valueOf(stateLock));
	    	} else {
	    		_log.info("Unsuccessful turn the muffle lock to " + String.valueOf(stateLock));
	    		while (!stateLock.equals(MQTT.stateLockStr)) {
	    			MQTT.publish("data/muffle", stateLock);
	    			ThreadUtil.milliSleep(3000);  
	    		}
	    	}
	    } catch (Exception e) {
			_log.info(e.getMessage());
	    }
	}
	
	
	public void openFumeCupboard(Tool gripper, ITaskLogger _log, IApplicationControl getAplControl) throws IOException, UnknownHostException, MqttException {
		try {
			_log.info("Opening fume cupboard started");
        	MyGripper.gripperAction(17, _log, getAplControl);
		   
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker21_frame, 0.0, -96.0, -22.0, 0.0, 0.0, 0.0)).setCartVelocity(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker21_frame, 0.0, 181.1, -22.0, 0.0, 0.0, 0.0)).setCartVelocity(70),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker21_frame, 0.0, 49.1, 180.0, 0.0, 0.0, 0.0)).setCartVelocity(300)
			));
			
			MQTT.publish("state/fumeCupboardDoor", "1");
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
        } finally {
        	_log.info("Opening fume cupboard finished");
        }
	}
	
	
	public void closeFumeCupboard(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) throws IOException, UnknownHostException, MqttException {
		try {
			_log.info("Closing fume cupboard started");
        	MyGripper.gripperAction(17, _log, getAplControl);

            gripper.getFrame("/TCP1").move(ptp(CoordinatesTransformation.transformCoordsToFlange(marker21_frame, 0.0, 0.0, 86.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.5).setBlendingCart(30));
            
            IMotionContainer imc0 = gripper.getFrame("/TCP1").moveAsync(lin(CoordinatesTransformation.transformCoordsToFlange(marker21_frame, 0.0, -27.0, 10.0, 0.0, 0.0, 0.0)).setCartVelocity(20));
			while (!imc0.isFinished()) {
				sensorData = iiwa.getExternalForceTorque(iiwa.getFlange());
				double zForce = sensorData.getForce().getZ();
				if (Math.abs(zForce) > 15) {
					_log.info("Force in Z vector " + zForce);
					imc0.cancel();
				}
				ThreadUtil.milliSleep(5);
			}    

            MQTT.publish("state/fumeCupboardDoor", "0");
            
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
        } finally {
        	_log.info("Closing fume cupboard finished");
        }
	}
		
	
	public void openMuffleDoor(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) throws IOException, UnknownHostException, MqttException {
		try {
			_log.info("Opening the muffle door started");
			MyGripper.gripperAction(17, _log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, 0.0, 0.0, 100.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.6),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -49.0, 27.0, 100.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.6) //setBlendingCart(70)
					));
			
			IMotionContainer imc1 = gripper.getFrame("/TCP1").moveAsync(lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -49.0, 27.0, 22.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.05));
			while (!imc1.isFinished()) {
				sensorData = iiwa.getExternalForceTorque(iiwa.getFlange());
				double zForce = sensorData.getForce().getZ();
				if (Math.abs(zForce) > 20) {
					_log.info("Force in Z vector " + zForce);
					imc1.cancel();
				}
				ThreadUtil.milliSleep(5);
			}
			
			muffleLockAction("1", _log);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -49.0, 25.0, 124.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.3).setBlendingCart(30)
					));
			
			gripper.getFrame("/TCP1").moveAsync(lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -49.0, 25.0, 124.0, Math.toRadians(-180), 0.0, 0.0)).setJointVelocityRel(1.0));
			MyGripper.gripperAction(180, _log, getAplControl);
			muffleLockAction("0", _log);
		
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -49.0, -145.0, 124.0, Math.toRadians(-180), 0.0, 0.0)).setCartVelocity(500).setBlendingCart(150),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -116.0, -145.0, 124.0, Math.toRadians(-180), 0.0, 0.0)).setCartVelocity(500).setBlendingCart(100),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -116.0, -145.0, 124.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(1.0),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -81.0, -145.0, 124.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(1.0).setBlendingCart(50),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -81.0, 0.0, 124.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(0.6).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -81.0, 0.0, 42.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setCartVelocity(150).setBlendingCart(30), 
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -216.0, 0.0, 42.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setCartVelocity(100).setBlendingCart(20) 
			));
			
			muffleLockAction("1", _log); 
		
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -216.0, 0.0, 144.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(0.3).setBlendingCart(20), 
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -216.0, 54.0, 311.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(0.3).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -19.0, 54.0, 311.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(0.3).setBlendingCart(70),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -19.0, -10.0, 311.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(1.0).setBlendingCart(70),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -19.0, -10.0, 113.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(1.0).setBlendingCart(70),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -19.0, 100.0, 113.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(0.3).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -30.0, 525.0, 360.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(0.3).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -30.0, 250.0, 360.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(1.0).setBlendingCart(50)
			));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
        } finally {
        	_log.info("Opening the muffle door finished");
        }
	}
	
	
	public void closeMuffleDoor(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) throws IOException, UnknownHostException, MqttException {
		try {
			_log.info("Closing the muffle door started");
			MyGripper.gripperAction(180, _log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, 3.0, 0.0, 360.0, Math.toRadians(-180), Math.toRadians(-45), 0.0)).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, 10.0, 525.0, 360.0, Math.toRadians(-180), Math.toRadians(-30), 0.0)).setJointVelocityRel(1.0).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, 10.0, 575.0, 274.0, Math.toRadians(-180), Math.toRadians(-30), 0.0)).setJointVelocityRel(0.4).setBlendingCart(30),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -24.0, 575.0, 274.0, Math.toRadians(-180), Math.toRadians(-30), 0.0)).setJointVelocityRel(0.4).setBlendingCart(10),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -24.0, 165.0, 274.0, Math.toRadians(-180), Math.toRadians(-30), 0.0)).setJointVelocityRel(0.4).setBlendingCart(150),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -19.0, 25.0, 46.0, Math.toRadians(-180), Math.toRadians(-30), 0.0)).setJointVelocityRel(0.4),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -19.0, 25.0, 46.0, Math.toRadians(-180), 0.0, 0.0)).setJointVelocityRel(1.0)
			));
			IMotionContainer imc2 = gripper.getFrame("/TCP1").moveAsync(lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -19.0, 25.0, 24.0, Math.toRadians(-180), 0.0, 0.0)).setJointVelocityRel(0.05));
			while (!imc2.isFinished()) {
				sensorData = iiwa.getExternalForceTorque(iiwa.getFlange());
				double zForce = sensorData.getForce().getZ();
				if (Math.abs(zForce) > 20) {
					_log.info("Force in Z vector " + zForce);
					imc2.cancel();
				}
				ThreadUtil.milliSleep(5);
			}
			gripper.getFrame("/TCP1").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -19.0, 25.0, 250.0, Math.toRadians(-180), 0.0, 0.0)).setJointVelocityRel(0.4).setBlendingCart(20));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
        } finally {
        	_log.info("Closing the muffle door finished");
        }
	}
	
	
	public void putTareInMuffle(Tool gripper, ITaskLogger _log, IApplicationControl getAplControl) throws IOException, UnknownHostException, MqttException {
		try {
			_log.info("Put the tare in the muffle started");
			
			MyGripper.gripperAction(120, _log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
					ptp(aboveTare).setJointVelocityRel(0.3).setBlendingCart(50),
					lin(takeTare).setJointVelocityRel(0.3)
			));
			MyGripper.gripperAction(75, _log, getAplControl);
			ThreadUtil.milliSleep(500);
			gripper.getFrame("/TCP1").move(batch(
					lin(aboveTare).setJointVelocityRel(0.3).setBlendingCart(50),
					lin(intermediatePoint1).setJointVelocityRel(0.3).setBlendingCart(300),
					ptp(intermediatePoint2).setJointVelocityRel(0.3).setBlendingCart(300),
					lin(inFrontOfMuffle).setCartVelocity(300).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -222.25, 236.93, 153.52, Math.toRadians(-92.82), 0.0, Math.toRadians(-19.48))).setCartVelocity(100).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -222.25, 236.93, -114.52, Math.toRadians(-92.82), 0.0, Math.toRadians(-19.48))).setCartVelocity(100),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -228.05, 236.93, -114.52, Math.toRadians(-92.82), 0.0, Math.toRadians(-19.48))).setCartVelocity(10)
			));
			MyGripper.gripperAction(120, _log, getAplControl);
			ThreadUtil.milliSleep(500);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -222.25, 236.93, -114.52, Math.toRadians(-92.82), 0.0, Math.toRadians(-19.48))).setCartVelocity(10),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -222.25, 236.93, 153.52, Math.toRadians(-92.82), 0.0, Math.toRadians(-19.48))).setCartVelocity(200).setBlendingCart(50),
					lin(inFrontOfMuffle).setCartVelocity(200).setBlendingCart(50)
			));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
        } finally {
        	_log.info("Put the tare in the muffle finished");
        }
	}
	
	
	public void takeTareOutMuffle(Tool gripper, ITaskLogger _log, IApplicationControl getAplControl) throws IOException, UnknownHostException, MqttException {
		try {
			_log.info("Take the tare out the muffle started");
			MyGripper.gripperAction(120, _log, getAplControl);
			
			gripper.getFrame("/TCP1").move(batch(
					lin(inFrontOfMuffle).setCartVelocity(100).setBlendingCart(50),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -222.25, 236.93, 153.52, Math.toRadians(-92.82), 0.0, Math.toRadians(-19.48))).setCartVelocity(200),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -222.25, 236.93, -114.52, Math.toRadians(-92.82), 0.0, Math.toRadians(-19.48))).setCartVelocity(100),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -228.05, 236.93, -114.52, Math.toRadians(-92.82), 0.0, Math.toRadians(-19.48))).setCartVelocity(10)
			));
			MyGripper.gripperAction(75, _log, getAplControl);
			ThreadUtil.milliSleep(500);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -222.25, 236.93, -114.52, Math.toRadians(-92.82), 0.0, Math.toRadians(-19.48))).setCartVelocity(10),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker22_frame, -222.25, 236.93, 153.52, Math.toRadians(-92.82), 0.0, Math.toRadians(-19.48))).setCartVelocity(100),
					lin(inFrontOfMuffle).setCartVelocity(100).setBlendingCart(50),
					lin(intermediatePoint2).setJointVelocityRel(0.3).setBlendingCart(300),
					lin(intermediatePoint1).setJointVelocityRel(0.3).setBlendingCart(300),
					lin(aboveTare).setJointVelocityRel(0.3).setBlendingCart(50),
					lin(takeTare).setJointVelocityRel(0.1)
			));
			MyGripper.gripperAction(120, _log, getAplControl);
			ThreadUtil.milliSleep(500);
			gripper.getFrame("/TCP1").move(lin(aboveTare).setCartVelocity(100).setBlendingCart(50));
				
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
        } finally {
        	_log.info("Take the tare out the muffle finished");
        }
	}
	
	
	public void turnOnMuffle(ITaskLogger _log) throws IOException, UnknownHostException, MqttException {
		try {
			_log.info("Turning on the muffle started");
			MQTT.publish("data/muffle_relay", "1");
			ThreadUtil.milliSleep(400);
			if ("1".equals(MQTT.stateRegulatorStr)) {
		    	_log.info("Successful turn the regulator ON");
			} else {
		    	_log.info("Unsuccessful turn the regulator ON");
		    	while ("1".equals(MQTT.stateRegulatorStr)) {
			    	MQTT.publish("data/muffle_relay", "1");
			    	_log.info("Trying to turn the regulator ON ...");
			    	ThreadUtil.milliSleep(3000);
		    	}
	    	}
		} catch (Exception e) {
			_log.info(e.getMessage());
		} finally {
        	_log.info("Turning on the muffle finished");
        }
	}
		
		
}





