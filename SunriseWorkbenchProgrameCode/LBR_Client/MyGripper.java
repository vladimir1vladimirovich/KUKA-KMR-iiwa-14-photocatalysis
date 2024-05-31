package photocatalysis;

import org.eclipse.paho.client.mqttv3.MqttException;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.task.ITaskLogger;


public class MyGripper {
	
	private static int angle;
	private static long time;
	
	public static int getAngle() {
		angle = MQTT.data_gripepr_angle;
		return angle;
	}
	
	
	public static void gripperAction(int angle, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.data_gripepr_angle = 0;
			MQTT.publish("data/gripper/rotate", String.valueOf(angle));
			time = System.currentTimeMillis();
			while (MQTT.data_gripepr_angle != angle) {
				if ((System.currentTimeMillis()-time)/1000 >= 2) {
					_log.info("Unsuccessful action gripper into " + String.valueOf(angle));
					_log.info("Try to move gripper into " + String.valueOf(angle) + " ...");
					MQTT.publish("data/gripper/rotate", String.valueOf(angle));
					time = System.currentTimeMillis();
				}
				ThreadUtil.milliSleep(50);
			}
			_log.info("Successful action gripper into " + String.valueOf(angle));
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public static void gripperActionSmoothly(int startingAngle, int finishingAngle, int speedDelay, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/gripper/rotate", Integer.toString(angle));
			ThreadUtil.milliSleep(500);			
			if (MQTT.data_gripepr_angle == angle) {
				_log.info("Successful action gripper into " + String.valueOf(angle));
			} else {
				_log.info("Unsuccessful action gripper into " + String.valueOf(angle));
				while (MQTT.data_gripepr_angle != angle) {
					MQTT.publish("data/gripper/rotate", Integer.toString(angle));
					_log.info("Try to move gripper into " + String.valueOf(angle) + " ...");
					ThreadUtil.milliSleep(3000);	
				}
			}
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	
}
