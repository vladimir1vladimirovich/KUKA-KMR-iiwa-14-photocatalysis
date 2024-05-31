package photocatalysis;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.batch;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.task.ITaskLogger;


public class TableKuka_actions {

	Frame above_stand = new Frame(153.57108640462636, -668.4614177575485, 312.94180789121043, -Math.PI, 0.0, Math.PI);
	Frame above_stand2 = new Frame(259.76294241217244, -668.4459573619426, 158.05814365060397, -Math.PI, 0.0, Math.PI);
	Frame in_stand = new Frame (153.57108640462636, -668.4614177575485, -57.218628423172134, -Math.PI, 0.0, Math.PI);
	Frame in__front_of_stand = new Frame (259.6982074009004, -668.4697743944885, -57.20678629811394, -Math.PI, 0.0, Math.PI);
		
	Frame middle_point = new Frame(-136.99966449172413, -460.93127166444356, 213.03677748421427, 1.5599670307327451, 0.0044748130658174914, -3.1393368488641133);
	Frame for_dispencer_1 = new Frame(-246.0016111204809, -674.6612171919583, 345.37896020387836, 0.029426317217356182, -0.02158827004426923, 1.5656830534537816);
	Frame above_dispencer_ml = new Frame(-341.0442117586518, -727.3335304078647, 342.5879671238586, 0.029474353577762673, -0.021482240762723585, 1.565396675438695);
	Frame dispencer_ml = new Frame(-403.994715426776, -727.3353041218475, 342.60336027693177, 0.02954483110887851, -0.0215010835232887, 1.56546989899659);
	Frame dispencer_ml_top = new Frame(-403.994715426776, -727.3353041218475, 618.9748476539523, 0.02954483110887851, -0.0215010835232887, 1.56546989899659);
	
	Frame for_dispencer_2 = new Frame(-149.6981927841731, -533.9752296780005, 347.12204069850696, -1.5706885396252235, -0.02478498494128766, 1.5778798621085255);
	Frame above_stand_dropping = new Frame(-471.2350198621567, -619.5341050292004, 342.4213681003719, -1.5707097800365877, -0.02428919375123063, 1.5780915255829457);
	Frame dispencer_ml2 = new Frame(-471.2929456922289, -659.2995332072777, 342.4530756255471, -1.5707637592378338, -0.024347492859697315, 1.5779561358344882);
	Frame dispencer_ml_top2 = new Frame(-471.2929456922289, -659.2995332072777, 618.9748476539523, -1.5707637592378338, -0.024347492859697315, 1.5779561358344882);
	
	Frame detectMarkerFrame52 = new Frame(223.7416793789604, -621.4197442558658, 423.60645695597634, Math.PI, 0.0, Math.PI);
	Frame aboveCuvetteFrame = new Frame(-51.05778239908538, -612.7861067209467, 423.69420904028334, Math.PI, 0.0, Math.PI);
	Frame Up1 = new Frame(-45.415054475653506, -586.9548704836084, 1026.939429883671, 0.012447489939639603, -3.4643422808537715E-4, 1.5763510304452053);
	
	JointPosition jP1 = new JointPosition(-2.25699836421587, -0.518932457965096, 0.6051991033577367, -2.0633066847097727, 2.0910415967487506, 0.19812026998704374, -0.23903183725116978);
	JointPosition jP11 = new JointPosition(-1.5355232774808796, 0.06816501290654826, -0.1523938007380864, -1.7386927411425455, 2.5639143760398277, 0.27638798447858104, -1.0026341708883324);
	JointPosition jP10 = new JointPosition(-0.6541359067982029, 0.10142926646408694, -0.9474302770216332, -1.7441666556728188, 2.883607860545395, 0.2419417268654987, -1.2395878836508947);
	JointPosition jP7 = new JointPosition(-0.4985761733720276, 1.0863880838646678, -1.829938771948467, -0.9717877348153847, 1.4193271330121602, 0.4746146743896021, 1.4446265268736054);
	
	
	private Frame marker52_frame;
	
	
	public void initialize(Tool gripper, LBR iiwa, ITaskLogger _log) {
		try {
			_log.info("TableKuka initialization");
			MQTT.initializeMQTT("TableKuka");
			MQTT.subscribe("state/transformCoords");
			MQTT.subscribe("state/gripper/rotate");
			MQTT.subscribe("state/gripper/rotate_smoothly");
		} catch (Exception e) {
			_log.info(e.getMessage());
		}
		gripper.attachTo(iiwa.getFlange());
	}
	
	
	public void puttingOnAdaptors(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MyGripper.gripperAction(86, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("TCP1").move(ptp(above_stand).setJointVelocityRel(0.1));
        	gripper.getFrame("TCP1").move(lin(above_stand2).setJointVelocityRel(0.1));
        	gripper.getFrame("TCP1").move(lin(in__front_of_stand).setJointVelocityRel(0.1));
        	gripper.getFrame("TCP1").move(lin(in_stand).setJointVelocityRel(0.01));
			gripper.getFrame("TCP1").move(lin(above_stand).setJointVelocityRel(0.05));
				
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	

	public void dosingMl(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			_log.info("start dosing");
			gripper.getFrame("TCP1").move(lin(middle_point).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(ptp(for_dispencer_1).setJointVelocityRel(0.1));
	        MyGripper.gripperAction(180, _log, getAplControl);
	        ThreadUtil.milliSleep(1000);
	        gripper.getFrame("TCP1").move(lin(above_dispencer_ml).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(lin(dispencer_ml).setJointVelocityRel(0.05));
	        gripper.getFrame("TCP1").move(lin(dispencer_ml_top).setJointVelocityRel(0.05));
	        MyGripper.gripperActionSmoothly(180, 105, 100, _log, getAplControl);
	        ThreadUtil.milliSleep(15000);
	        MyGripper.gripperActionSmoothly(105, 180, 100, _log, getAplControl);
	        ThreadUtil.milliSleep(15000);
	        gripper.getFrame("TCP1").move(lin(dispencer_ml_top).setJointVelocityRel(0.05));
	        gripper.getFrame("TCP1").move(lin(dispencer_ml).setJointVelocityRel(0.05));
	        gripper.getFrame("TCP1").move(lin(above_dispencer_ml).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(ptp(for_dispencer_1).setJointVelocityRel(0.1));
	        
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	

	public void droppingMl(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			_log.info("start dropping");
			gripper.getFrame("TCP1").move(lin(middle_point).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(ptp(for_dispencer_2).setJointVelocityRel(0.1));
	        MyGripper.gripperAction(180, _log, getAplControl);
	        ThreadUtil.milliSleep(1000);
	        gripper.getFrame("TCP1").move(ptp(above_stand_dropping).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(ptp(dispencer_ml2).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(ptp(dispencer_ml_top2).setJointVelocityRel(0.1));
	        MyGripper.gripperActionSmoothly(180, 145, 100, _log, getAplControl);
	        ThreadUtil.milliSleep(15000);
	        MyGripper.gripperActionSmoothly(145, 180, 100, _log, getAplControl);
	        ThreadUtil.milliSleep(15000);
	        gripper.getFrame("TCP1").move(ptp(dispencer_ml2).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(ptp(above_stand_dropping).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(ptp(for_dispencer_2).setJointVelocityRel(0.1));
	        
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
		

	public void dosingCuvette(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("TCP1").move(ptp(aboveCuvetteFrame).setJointVelocityRel(0.5));
			gripper.getFrame("TCP1").move(lin(detectMarkerFrame52).setJointVelocityRel(0.5));
			ThreadUtil.milliSleep(2000);
			marker52_frame = MQTT.getMarkerFrame(52, 30, iiwa);
			gripper.getFrame("TCP1").move(ptp(aboveCuvetteFrame).setJointVelocityRel(0.5));
			gripper.getFrame("TCP1").move(lin(middle_point).setJointVelocityRel(0.4));
	        gripper.getFrame("TCP1").move(ptp(for_dispencer_1).setJointVelocityRel(0.2));
	        MyGripper.gripperAction(180, _log, getAplControl);
	        ThreadUtil.milliSleep(1000);
	        gripper.getFrame("TCP1").move(lin(above_dispencer_ml).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(lin(dispencer_ml).setJointVelocityRel(0.05));
	        gripper.getFrame("TCP1").move(lin(dispencer_ml_top).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(ptp(jP10).setJointVelocityRel(0.1));
	        gripper.getFrame("TCP1").move(ptp(jP7).setJointVelocityRel(0.1));

		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
}
