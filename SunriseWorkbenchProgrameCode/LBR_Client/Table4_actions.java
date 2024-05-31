package photocatalysis;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.batch;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import java.io.IOException;
import java.net.UnknownHostException;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.OrientationReferenceSystem;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.task.ITaskLogger;



public class Table4_actions {
	private Frame boxP1 = new Frame(132.85775795978722, -295.4216566133544, 303.21469142172896, Math.PI, 0.0, Math.PI);
	private Frame detectBox = new Frame(578.9342122830521, -298.580501144287, 387.4100873621436, Math.PI, 0.0, Math.PI);
	
	private Frame containerP1 = new Frame(372.7379253684842, -128.93017558266126, 357.22224490407524, -Math.PI, 0.0, Math.PI);
	private Frame detectContainer = new Frame(511.1226181091371, 81.59258507160854, 244.68397640375687, -Math.PI, 0.0, Math.PI);
	
	private Frame cuvetteP1 = new Frame(331.47504426377446, -128.1846895411616, 444.6720043562472, -Math.PI, 0.0, Math.PI);
	private Frame detectCuvette = new Frame(312.91260098388193, -135.65114395439846, 251.76088625337877, -Math.PI, 0.0, Math.PI);
	
	private Frame spphP1 = new Frame(209.3383305358418, -315.4009812594895, 335.8645418550668, -Math.PI, 0.0, Math.PI);
	private Frame spphP2 = new Frame(-10.87498795039386, -646.6853332813846, 335.87254447762166, -Math.PI, 0.0, Math.PI);
	private Frame spphP3 = new Frame(217.10632396380336, -632.1775534928369, 254.2283461939668, 0.0, Math.PI/2, 0.0);
	private Frame detectSpph = new Frame (287.6288889191311, -632.1794287579538, 254.23019222365224, 0.0, Math.PI/2, 0.0);
	
	private Frame generalPoint = new Frame(215.9917400370232, -287.7538095872579, 475.0173907876821, -Math.PI, 0.0, Math.PI);
	private Frame generalPointInFrontOfSpph = new Frame(190.6388389866642, -565.403381187973, 738.9510781199198,  0.0, Math.PI/2, 0.0);
	
	private Frame detectSpphInsideBefore = new Frame (365.3879761526573, -536.3623255173103, 481.70807925646807, 3.1230279665405094, -1.473355625795614E-4, -3.141424914166553);
	private Frame detectSpphInside = new Frame (401.78425375981146, -544.6928583950634, 417.65419860079317, -Math.PI, 0.0, Math.PI);
	
	private Frame glassStandPlatformUp = new Frame(-435.50958369416674, -512.7771705302214, 235.163733777917, 2.5419661263150544, 1.5688763111658681, -2.8511667296342336);
	private Frame glassStandPlatformDown = new Frame(-435.4357208281991, -512.930024551879, 57.86541733511934, 2.6374171247991636, 1.5685539048886752, -2.755837339482935);
	private Frame generalPointB = new Frame(444.01986120104806, -287.6936324191161, 706.7747433778009, 0.0, Math.PI/2, 0.0);
	private Frame glassAboveBox = new Frame(346.8100715087987, -306.2082180310563, 720.1018149342511, 0.0, Math.PI/2, 0.0);
	private Frame glassInFrontOfBox = new Frame (648.2995962336547, -296.1358715981232, 370.8093169815731, 0.5466026741845996, 1.548200329176132, 0.5641631383445347);
	private Frame capStandPlatformUp = new Frame(-95.76354045170973, -692.8723432844776, 282.0123089706877, -1.0772558427457162, 1.5692879780134974, -0.21482936930523772);
	private Frame capStandPlatformDown = new Frame(-96.5650373502636, -691.1440804128412, 99.77764005951599, -1.6534560925661308, 1.5696177014738666, -0.7909675507930866);
	
	private	Frame above_stand = new Frame(155.49127052592897, -668.4425686663964, 308.4293069378316, 3.130145728509885, 3.8271076379116465E-5, 3.141571162570121);
	private	Frame above_stand2 = new Frame(222.39785089393428, -668.3067243609563, -55.75915232244378,  -Math.PI, 0.0, Math.PI);
	private	Frame in_stand = new Frame (153.57108640462636, -668.4614177575485, -57.218628423172134, -Math.PI, 0.0, Math.PI);
	private	Frame in_front_of_stand = new Frame (219.6982074009004, -668.4697743944885, -57.20678629811394, -Math.PI, 0.0, Math.PI);
	
	private Frame above_stand_remove_adaptors = new Frame(155.42924467463598, -668.3803511589568, 70.411831951140414, 3.1300974313581533, -2.409334279090282E-5, 3.1413758793406785);
	private Frame in_stand_remove_adaptors = new Frame(155.50218327044232, -668.3499458377775, -55.43603622516059, 3.1300376530807266, 5.407005077980879E-5, 3.1413383626874274);
	private Frame in_front_of_stand_remove = new Frame(198.04610876736302, -668.3437720161752, -55.46601604898558, 3.129940078325032, 2.157175061309644E-5, 3.141298844784355);
	private Frame above_stand_remove_adaptors2 = new Frame(198.04610876736302, -668.3437720161752, 435.8535540855372, 3.129940078325032, 2.157175061309644E-5, 3.141298844784355);
	
	private	Frame middle_point = new Frame(-136.99966449172413, -460.93127166444356, 213.03677748421427, 1.5599670307327451, 0.0044748130658174914, -3.1393368488641133);
	private	Frame for_dispencer_1 = new Frame(-246.0016111204809, -674.6612171919583, 345.37896020387836, 0.029426317217356182, -0.02158827004426923, 1.5656830534537816);
	private	Frame above_dispencer_ml = new Frame(-341.0442117586518, -727.3335304078647, 342.5879671238586, 0.029474353577762673, -0.021482240762723585, 1.565396675438695);
	private	Frame dispencer_ml = new Frame(-403.994715426776, -727.3353041218475, 342.60336027693177, 0.02954483110887851, -0.0215010835232887, 1.56546989899659);
	private	Frame dispencer_ml_top = new Frame(-403.994715426776, -727.3353041218475, 618.9748476539523, 0.02954483110887851, -0.0215010835232887, 1.56546989899659);

	private JointPosition jP1 = new JointPosition(-0.6541359067982029, 0.10142926646408694, -0.9474302770216332, -1.7441666556728188, 2.883607860545395, 0.2419417268654987, -1.2395878836508947);
	private JointPosition jP2 = new JointPosition(-0.4986424461417197, 1.0863959934786718, -1.8300058836761142, -0.971779046239119, 1.4194258830431064, 0.4744980679605286, 1.4445984837958492);
	private	Frame jP3 = new Frame(413.95457053056975, -439.1519525612182, 811.6242430597235, 0.030165032729728095, 8.000504868149415E-4, 1.5750190604192178);
	
	private Frame takeTare = new Frame (90.49682874037673, -568.0299938861223, 59.20442316880077, 3.016816050398784, 1.276023614073105, 3.013896860742084);
	private Frame aboveTare = new Frame (90.5979625993402, -567.9260225541044, 139.13344203802754, 3.016816050398784, 1.276023614073105, 3.013896860742084);
	private Frame tareAboveBox = new Frame(201.6159883313915, -308.58122361010885, 680.2568670121111, 3.1314317932433076, 1.308873282550191, 3.1414205635143815);
	private Frame tareAboveSpph = new Frame (488.5021073994891, -477.94469943288, 487.54808655578, 3.1315258787090783, 1.3089841301266294, 3.1414921373695166);
	
	private Frame aboveSpph= new Frame(197.943368901523, -668.261798768558, 435.8810001364981, 3.1295877298229997, -1.310541267516568E-4, 3.1411900372589145);
	private Frame leftOfOpenSpphDoor= new Frame (498.75073519639744, -399.28266350322895, 693.6814963894149, 3.0007034945828392, 1.2659369350873622, -1.7935980022562277);
	
	private Frame marker51_frame;
	private Frame marker52_frame;
	private Frame marker53_frame;
	private Frame marker54_frame;
	private Frame marker55_frame;
	
	private ForceSensorData sensorData;
	private double xForce;
	private double yForce;
	private double zForce;
	
	long time1;
	
	
	public void initialize(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			_log.info("Table4 initialization");
			MQTT.initializeMQTT("Table4");
			MQTT.subscribe("state/transformCoords");
			MQTT.subscribe("state/gripper/rotate");
			MQTT.subscribe("state/box/initialize");
			MQTT.subscribe("state/box/front_door");
			MQTT.subscribe("state/box/upper_door"); 
			MQTT.subscribe("state/box/stirrer/power"); 
			MQTT.subscribe("state/box/uv_led/power"); 
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
		gripper.attachTo(iiwa.getFlange());
	}
	

	public void detectMarkersAndInitializationFrames(int[] markers_id, Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			for (int m_id: markers_id) {
				if (m_id == 51) {
					_log.info("Scanning the 51 marker");
					gripper.getFrame("/TCP1").move(batch(
							ptp(boxP1).setJointVelocityRel(0.5),
							lin(detectBox).setJointVelocityRel(0.5)
							));
					ThreadUtil.milliSleep(3000);
					marker51_frame = MQTT.getMarkerFrame(51, 30, iiwa);
				}
				if (m_id == 52) {
					_log.info("Scanning the 52 marker");
					gripper.getFrame("/TCP1").move(batch(
							ptp(cuvetteP1).setJointVelocityRel(0.5),
							ptp(detectCuvette).setJointVelocityRel(0.5)
							));
					ThreadUtil.milliSleep(3000);
					marker52_frame = MQTT.getMarkerFrame(52, 30, iiwa);	
				}
				if (m_id == 53) {
					_log.info("Scanning the 53 marker");
					gripper.getFrame("/TCP1").move(batch(
							ptp(detectSpphInsideBefore).setJointVelocityRel(0.5),
							ptp(detectSpphInside).setJointVelocityRel(0.5)
							));
					ThreadUtil.milliSleep(3000);
					marker53_frame = MQTT.getMarkerFrame(53, 30, iiwa);
					gripper.getFrame("/TCP1").move(ptp(detectSpphInsideBefore).setJointVelocityRel(0.5));
				}
				if (m_id == 54) {
					_log.info("Scanning the 54 marker");
					gripper.getFrame("/TCP1").move(batch(
							ptp(spphP1).setJointVelocityRel(0.5),
							ptp(spphP2).setJointVelocityRel(0.5),
							ptp(spphP3).setJointVelocityRel(0.5),
							ptp(detectSpph).setJointVelocityRel(0.5)
							));
					ThreadUtil.milliSleep(3000);
					marker54_frame = MQTT.getMarkerFrame(54, 30, iiwa);
					gripper.getFrame("TCP1").move(ptp(generalPointInFrontOfSpph).setJointVelocityRel(0.7));
				}
				if (m_id == 55) {
					_log.info("Scanning the 55 marker");
					gripper.getFrame("/TCP1").move(batch(
							ptp(containerP1).setJointVelocityRel(0.5),
							ptp(detectContainer).setJointVelocityRel(0.5)
							));
					ThreadUtil.milliSleep(3000);
					marker55_frame = MQTT.getMarkerFrame(55, 30, iiwa);	
				}
			}	
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	

	public void moveToGeneralPoint(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			
			gripper.getFrame("TCP1").move(ptp(generalPoint).setJointVelocityRel(0.5));
	        
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}	
	
	
	public void returnDispenserMl(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MyGripper.gripperAction(180, _log, getAplControl);
		    ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					ptp(jP2).setJointVelocityRel(1.0),
					ptp(jP1).setJointVelocityRel(1.0),
					lin(dispencer_ml_top).setJointVelocityRel(1.0),
					lin(dispencer_ml).setJointVelocityRel(0.05),
					lin(above_dispencer_ml).setJointVelocityRel(0.3),
					ptp(for_dispencer_1).setJointVelocityRel(1.0),
					lin(middle_point).setJointVelocityRel(1.0)
					));
	        
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}	

		
	public void doseMl(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("/TCP_D").move(batch(
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, -120.0, 550.0, Math.toRadians(-180), Math.toRadians(90), 0.0)).setJointVelocityRel(1.0),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, -120.0, 550.0, Math.toRadians(-150), Math.toRadians(90), 0.0)).setJointVelocityRel(1.0),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 550.0, Math.toRadians(-150), Math.toRadians(90), 0.0)).setJointVelocityRel(1.0)
					));
			MyGripper.gripperActionSmoothly(180, 105, 100, _log, getAplControl);
			ThreadUtil.milliSleep(8000);
			gripper.getFrame("/TCP_D").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 7.0, -5.0, 319.0, Math.toRadians(-150), Math.toRadians(90), 0.0)).setJointVelocityRel(0.1));
			MyGripper.gripperActionSmoothly(105, 180, 100, _log, getAplControl);
			gripper.getFrame("/TCP_D").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 7.0, -5.0, 330.0, Math.toRadians(-150), Math.toRadians(90), 0.0)).setCartVelocity(4));
			ThreadUtil.milliSleep(2000);
			gripper.getFrame("/TCP_D").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 550.0, Math.toRadians(-150), Math.toRadians(90), 0.0)).setJointVelocityRel(0.1),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, -120.0, 550.0, Math.toRadians(-180), Math.toRadians(90), 0.0)).setJointVelocityRel(0.1)
					));
			closeUpperDoor(_log, getAplControl);
			turnOnUvLed(_log, getAplControl);
			time1 = System.currentTimeMillis();
			gripper.getFrame("/TCP_D").move(batch(
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 4.0, -51.5, 470.0, Math.toRadians(-180), Math.toRadians(90), 0.0)).setJointVelocityRel(0.1),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 4.0, -51.5, 330.0, Math.toRadians(-180), Math.toRadians(90), 0.0)).setJointVelocityRel(0.05),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 4.0, -51.5, 322.0, Math.toRadians(-180), Math.toRadians(90), 0.0)).setJointVelocityRel(0.05)
					));
			MyGripper.gripperActionSmoothly(180, 105, 100, _log, getAplControl);
			gripper.getFrame("/TCP_D").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 5.0, -56.5, 311.0, Math.toRadians(-180), Math.toRadians(90), 0.0)).setCartVelocity(3));
			ThreadUtil.milliSleep(4000);
			gripper.getFrame("/TCP_D").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 5.0, -56.5, 550.0, Math.toRadians(-180), Math.toRadians(90), 0.0)).setJointVelocityRel(0.1));
			MyGripper.gripperActionSmoothly(105, 180, 50, _log, getAplControl);
			ThreadUtil.milliSleep(3000);			
			
		} catch (Exception e) {	
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	

	public void takeDispenserMl(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("/TCP1").move(batch(
					lin(middle_point).setJointVelocityRel(1.0),
					ptp(for_dispencer_1).setJointVelocityRel(1.0)
					));
	        MyGripper.gripperAction(180, _log, getAplControl);
	        ThreadUtil.milliSleep(1000);
	        gripper.getFrame("/TCP1").move(batch(
	        		lin(above_dispencer_ml).setJointVelocityRel(1.0),
	        		lin(dispencer_ml).setJointVelocityRel(0.05),
	        		lin(dispencer_ml_top).setJointVelocityRel(0.1),
	        		ptp(jP1).setJointVelocityRel(1.0),
	        		ptp(jP2).setJointVelocityRel(1.0),
	        		lin(jP3).setJointVelocityRel(1.0)
	        		));
	        
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void putOnAdaptors(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MyGripper.gripperAction(86, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					ptp(above_stand).setJointVelocityRel(1.0),
					lin(above_stand2).setJointVelocityRel(0.5),
					lin(in_front_of_stand).setJointVelocityRel(0.05),
					lin(in_stand).setJointVelocityRel(0.01),
					lin(above_stand).setJointVelocityRel(0.1)
					));
				
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	

	public void removeOnAdaptors(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			
			MyGripper.gripperAction(86, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					ptp(above_stand).setJointVelocityRel(1.0),
					lin(above_stand_remove_adaptors).setJointVelocityRel(1.0),
					lin(in_stand_remove_adaptors).setJointVelocityRel(0.05),
					lin(in_front_of_stand_remove).setJointVelocityRel(0.05),
					lin(above_stand_remove_adaptors2).setJointVelocityRel(1.0)
					));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
		
	
	public void moveCuvetteInSpectrophotometer(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MyGripper.gripperAction(180, _log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker52_frame, -40.0, 0.0, 70.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.3),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker52_frame, -40.0, 0.0, 30.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(46, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker52_frame, -40.0, 0.0, 158.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.3),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 0.0, -130.0, 110.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.2),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 0.0, -44.0, 40.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 0.0, -44.0, 5.0, 0.0, 0.0, 0.0)).setCartVelocity(5)
					));
			MyGripper.gripperAction(90, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 0.0, -44.0, 25.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.3));
			MyGripper.gripperAction(17, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			IMotionContainer imc1 = gripper.getFrame("/TCP1").moveAsync(lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 0.0, -44.0, 10.0, 0.0, 0.0, 0.0)).setCartVelocity(5));
			while (!imc1.isFinished()) {
				sensorData = iiwa.getExternalForceTorque(iiwa.getFlange());
				zForce = sensorData.getForce().getZ();
				if (Math.abs(zForce) > 17) {
					_log.info("Force in Z vector " + zForce);
					imc1.cancel();
				}
				ThreadUtil.milliSleep(5);
			}
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 0.0, -44.0, 40.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.3),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, 0.0, -130.0, 110.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.3)
					));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
	     }
	}
	
	
	public void returnCuvetteToStand(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker52_frame, -40.0, 0.0, 158.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.3),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker52_frame, -40.0, 0.0, 33.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.05)
					));
			MyGripper.gripperAction(90, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker52_frame, -40.0, 0.0, 158.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(1.0));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
	     }
	}
	
	
	public void drainLiquidFromCuvette(int sam_num, Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MyGripper.gripperAction(70, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -44.0, 40.0, 0.0, 0.0, 0.0)).setJointVelocityRel(1.0));
			IMotionContainer imc3 = gripper.getFrame("/TCP1").moveAsync(lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -44.0, 3.0, 0.0, 0.0, 0.0)).setCartVelocity(5));
			while (!imc3.isFinished()) {
				sensorData = iiwa.getExternalForceTorque(iiwa.getFlange());
				zForce = sensorData.getForce().getZ();
				if (Math.abs(zForce) > 17) {
					_log.info("Force in Z vector " + zForce);
					imc3.cancel();
				}
				ThreadUtil.milliSleep(5);
			}
			MyGripper.gripperAction(46, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -44.0, 40.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -130.0, 110.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.2),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker55_frame, -93.0, 0.0, 260.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.3), //x=-90
					lin(CoordinatesTransformation.transformCoordsToFlange(marker55_frame, -93.0, 0.0, 360.0, Math.toRadians(-105), 0.0, Math.toRadians(-130))).setJointVelocityRel(0.1),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker55_frame, -93.0, 0.0, 460.0, Math.toRadians(-105), 0.0, Math.toRadians(-130))).setJointVelocityRel(0.3),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker55_frame, -93.0, 0.0, 360.0, Math.toRadians(-105), 0.0, Math.toRadians(-130))).setJointVelocityRel(0.3),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker55_frame, -93.0, 0.0, 460.0, Math.toRadians(-105), 0.0, Math.toRadians(-130))).setJointVelocityRel(0.3),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker55_frame, -93.0, 0.0, 360.0, Math.toRadians(-105), 0.0, Math.toRadians(-130))).setJointVelocityRel(0.3),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker55_frame, -93.0, 0.0, 260.0, Math.toRadians(-90), 0.0, 0.0)).setJointVelocityRel(0.3)
					));
	
			if (sam_num == 20) {
				returnCuvetteToStand(gripper, iiwa, _log, getAplControl);
			} else {
				gripper.getFrame("/TCP1").move(batch(
						lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -130.0, 110.0, 0.0, 0.0, 0.0)).setJointVelocityRel(1.0),
						lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -44.0, 40.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1),
						lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -44.0, 10.0, 0.0, 0.0, 0.0)).setCartVelocity(5)
						));
				MyGripper.gripperAction(70, _log, getAplControl);
				ThreadUtil.milliSleep(1000);
				gripper.getFrame("/TCP1").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -44.0, 25.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1));
				MyGripper.gripperAction(17, _log, getAplControl);
				ThreadUtil.milliSleep(1000);
				IMotionContainer imc1 = gripper.getFrame("/TCP1").moveAsync(lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -44.0, 8.0, 0.0, 0.0, 0.0)).setCartVelocity(5));
				while (!imc1.isFinished()) {
					sensorData = iiwa.getExternalForceTorque(iiwa.getFlange());
					zForce = sensorData.getForce().getZ();
					if (Math.abs(zForce) > 17) {
						_log.info("Force in Z vector " + zForce);
						imc1.cancel();
					}
					ThreadUtil.milliSleep(5);
				}
				gripper.getFrame("/TCP1").move(batch(
						lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -44.0, 40.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1),
						lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -1.0, -130.0, 110.0, 0.0, 0.0, 0.0)).setJointVelocityRel(1.0)
						));
				MyGripper.gripperAction(180, _log, getAplControl);
			}
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
	    }
	}
	
	
	public void openSpectrophotometerDoor(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("/TCP1").move(ptp(generalPointInFrontOfSpph).setJointVelocityRel(1.0));
			MyGripper.gripperAction(17, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker54_frame, -100.0, 0.0, 120.0, 0.0, 0.0, 0.0)).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker54_frame, -100.0, 0.0, 16.0, 0.0, 0.0, 0.0)).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker54_frame, -100.0, 29.0, 16.0, 0.0, 0.0, 0.0)).setJointVelocityRel(0.1)
					));
			Frame P1 = CoordinatesTransformation.transformCoordsToFlange(marker54_frame, -100.0, 299.0, -49.0, 0.0, 0.0, 0.0);
			Frame P2 = CoordinatesTransformation.transformCoordsToFlange(marker54_frame, -100.0, 430.0, -302.0, 0.0, 0.0, 0.0);
			gripper.getFrame("/TCP1").move(circ(P1, P2).setJointVelocityRel(0.3));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
				
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
	    }
	}
	
	
	public void closeSpectrophotometerDoor(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					ptp(aboveSpph).setJointVelocityRel(1.0),
					lin(leftOfOpenSpphDoor).setJointVelocityRel(1.0),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -130.0, 0.0, 330.0, 0.0, Math.toRadians(-90), Math.toRadians(76))).setJointVelocityRel(1.0),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -130.0, 189.0, 330.0, 0.0, Math.toRadians(-90), Math.toRadians(76))).setJointVelocityRel(1.0),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -130.0, 202.0, 364.0, Math.toRadians(5), Math.toRadians(-90), Math.toRadians(76))).setJointVelocityRel(1.0),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -55.0, 202.0, 383.0,  Math.toRadians(5), Math.toRadians(-90), Math.toRadians(76))).setJointVelocityRel(0.3),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -55.0, 189.0, 383.0,   Math.toRadians(5), Math.toRadians(-90), Math.toRadians(76))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -15.0, 58.0, 425.0,   Math.toRadians(5), Math.toRadians(-90), Math.toRadians(112))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -15.0, 58.0, 440.0,   Math.toRadians(5), Math.toRadians(-90), Math.toRadians(180))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -15.0, 58.0, 440.0,  Math.toRadians(-90), Math.toRadians(90), Math.toRadians(0))).setJointVelocityRel(1.0)
					));
				
			Frame P1 = CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -15.0, -148.0, 271.0,  Math.toRadians(-90), Math.toRadians(90), Math.toRadians(0));
			Frame P2 = CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -15.0, -178.0, 58.0,  Math.toRadians(-90), Math.toRadians(90), Math.toRadians(0));
			gripper.getFrame("/TCP1").move(batch(
					circ(P1, P2)
					.setOrientationReferenceSystem(OrientationReferenceSystem.Base)
					.setOrientationType(SplineOrientationType.Constant).setJointVelocityRel(1.0),
			
					lin(CoordinatesTransformation.transformCoordsToFlange(marker53_frame, -15.0, -298.0, 58.0,  Math.toRadians(-90), Math.toRadians(90), Math.toRadians(0))).setJointVelocityRel(1.0)
					));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
	    }
	}
	
	
	public void pressEnter(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			gripper.getFrame("/TCP1").move(ptp(generalPointInFrontOfSpph).setJointVelocityRel(1.0));
			MyGripper.gripperAction(17, _log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker54_frame, 130.0, 100.0, 100.0, 0.0, 0.0, 0.0)).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker54_frame, 141.0, 80.0, -56.0, 0.0, Math.toRadians(-45), Math.toRadians(-60))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker54_frame,  141.0, 72.5, -56.0, 0.0, Math.toRadians(-45), Math.toRadians(-60))).setJointVelocityRel(0.005),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker54_frame, 141.0, 100.0, -56.0, 0.0, Math.toRadians(-45), Math.toRadians(-60))).setJointVelocityRel(1.0)
					));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
	    }
	}
	
	
	public void actionWithTare(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MyGripper.gripperAction(120, _log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
				ptp(aboveTare).setJointVelocityRel(1.0).setBlendingCart(50),
				lin(takeTare).setJointVelocityRel(0.3)
			));
			MyGripper.gripperAction(77, _log, getAplControl);
			ThreadUtil.milliSleep(500);
			gripper.getFrame("/TCP1").move(batch(
					lin(aboveTare).setJointVelocityRel(0.1),
					ptp(tareAboveBox).setJointVelocityRel(0.1),
					lin(tareAboveSpph).setJointVelocityRel(0.1)
					));
			turnOffUvLed(_log, getAplControl);
			openFrontDoor(_log, getAplControl);
			ThreadUtil.milliSleep(7000);
			gripper.getFrame("/TCP1").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, -65.0, 110.0, 0.0, 0.0,Math.toRadians(75))).setJointVelocityRel(0.1),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, -2.0, -6.0, 100.0, 0.0, 0.0,Math.toRadians(75))).setJointVelocityRel(0.1),	
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, -2.0, -6.0, 100.0, Math.toRadians(-180), 0.0,Math.toRadians(-75))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, -2.0, -6.0, 130.0, Math.toRadians(-180), 0.0,Math.toRadians(-75))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, -2.0, -6.0, 100.0, Math.toRadians(-180), 0.0,Math.toRadians(-75))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, -2.0, -6.0, 130.0, Math.toRadians(-180), 0.0,Math.toRadians(-75))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, -2.0, -6.0, 100.0, Math.toRadians(-180), 0.0,Math.toRadians(-75))).setJointVelocityRel(1.0),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, -2.0, -6.0, 100.0, 0.0, 0.0,Math.toRadians(75))).setJointVelocityRel(1.0),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, -65.0, 110.0, 0.0, 0.0,Math.toRadians(75))).setJointVelocityRel(0.4),
					ptp(tareAboveBox).setJointVelocityRel(0.4)
					));
			closeFrontDoor(_log, getAplControl);
			turnOnUvLed(_log, getAplControl);
			time1 = System.currentTimeMillis();
			gripper.getFrame("/TCP1").move(batch(
					ptp(aboveTare).setJointVelocityRel(0.4).setBlendingCart(50),
					lin(takeTare).setJointVelocityRel(0.3)
				));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(500);
			gripper.getFrame("/TCP1").move(lin(aboveTare).setJointVelocityRel(1.0));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
		
	
	public void actionWithGlass(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {			
			MyGripper.gripperAction(180, _log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
					ptp(glassStandPlatformUp).setJointVelocityRel(0.5),
					lin(glassStandPlatformDown).setJointVelocityRel(0.2)
					));
			MyGripper.gripperAction(97, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(batch(
					lin(glassStandPlatformUp).setJointVelocityRel(0.2),
					ptp(glassAboveBox).setJointVelocityRel(0.1),
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, -35.0, 95.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.1)
					));
			gripper.getFrame("/TCP3").move(batch(
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 80.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.1),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 48.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP3").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 94.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.3));
			gripper.getFrame("/TCP2").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 94.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.3));
			MyGripper.gripperAction(95, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP2").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 112.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.2));
			gripper.getFrame("/TCP1").move(batch(		
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, -55.0, 112.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.2),
					ptp(glassAboveBox).setJointVelocityRel(0.1)
					));
			closeFrontDoor(_log, getAplControl);
			turnOnStirrer(_log, getAplControl);
			turnOnUvLed(_log, getAplControl);
			gripper.getFrame("/TCP1").move(batch(
					ptp(capStandPlatformUp).setJointVelocityRel(0.3),
					lin(capStandPlatformDown).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(lin(capStandPlatformUp).setJointVelocityRel(0.3));
			
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void retaurnGlassToPlatform(Tool gripper, LBR iiwa, ITaskLogger _log, IApplicationControl getAplControl) {
		try {			
			gripper.getFrame("/TCP1").move(batch(		
					ptp(glassAboveBox).setJointVelocityRel(0.1),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, -55.0, 112.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.2)
					));
			gripper.getFrame("/TCP2").move(lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 112.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.2));
			gripper.getFrame("/TCP3").move(batch(
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 94.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.3),
					lin(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 48.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.1)
					));
			MyGripper.gripperAction(97, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP3").move(ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, 0.0, 80.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.1));
			gripper.getFrame("/TCP1").move(batch(
					ptp(CoordinatesTransformation.transformCoordsToFlange(marker51_frame, 0.0, -35.0, 95.0, 0.0, 0.0,Math.toRadians(90))).setJointVelocityRel(0.1),
					ptp(glassAboveBox).setJointVelocityRel(0.1),
					lin(glassStandPlatformUp).setJointVelocityRel(0.2),
					lin(glassStandPlatformDown).setJointVelocityRel(0.2)
					));		
			MyGripper.gripperAction(180, _log, getAplControl);
			ThreadUtil.milliSleep(1000);
			gripper.getFrame("/TCP1").move(ptp(glassStandPlatformUp).setJointVelocityRel(0.5));
				
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
		
	
	public void openFrontDoor(ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/box/front_door", "open");
			ThreadUtil.milliSleep(1000);
			if (MQTT.stateBoxFrontDoor.equals("open")) {
				_log.info("Successful opening the front door");
			} else {
				_log.info("Unsuccessful opening the front door");
				while (!MQTT.stateBoxFrontDoor.equals("open")) {
					MQTT.publish("data/box/front_door", "open");
					_log.info("Trying to open the front door ...");
					ThreadUtil.milliSleep(3000);	
				}
			}
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
		
	
	public void closeFrontDoor(ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/box/front_door", "close");
			ThreadUtil.milliSleep(1000);
			if (MQTT.stateBoxFrontDoor.equals("close")) {
				_log.info("Successful closing the front door");
			} else {
				_log.info("Unsuccessful closing the front door");
				while (!MQTT.stateBoxFrontDoor.equals("close")) {
					MQTT.publish("data/box/front_door", "close");
					_log.info("Trying to close the front door ...");
					ThreadUtil.milliSleep(3000);
				}
			}
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void openUpperDoor(ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/box/upper_door", "open");
			ThreadUtil.milliSleep(1000);
			if (MQTT.stateBoxUpperDoor.equals("open")) {
				_log.info("Successful opening the upper door");
			} else {
				_log.info("Unsuccessful opening the upper door");
				while (!MQTT.stateBoxUpperDoor.equals("open")) {
					MQTT.publish("data/box/upper_door", "open");
					_log.info("Trying to open the upper door ...");
					ThreadUtil.milliSleep(3000);	
				}
			}
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void closeUpperDoor(ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/box/upper_door", "close");
			ThreadUtil.milliSleep(1000);
			if (MQTT.stateBoxUpperDoor.equals("close")) {
				_log.info("Successful closing the upper door");
			} else {
				_log.info("Unsuccessful closing the upper door");
				while (!MQTT.stateBoxUpperDoor.equals("close")) {
					MQTT.publish("data/box/upper_door", "close");
					_log.info("Trying to close the upper door ...");
					ThreadUtil.milliSleep(3000);	
				}
			}
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void turnOnUvLed(ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/box/uv_led/power", "1");
			ThreadUtil.milliSleep(1000);
			if (MQTT.stateBoxUvLed.equals("1")) {
				_log.info("Successful turning on the UV led");
			} else {
				_log.info("Unsuccessful turning on the UV led");
				while (!MQTT.stateBoxUvLed.equals("1")) {
					MQTT.publish("data/box/uv_led/power", "1");
					_log.info("Trying to turn on the UV led ...");
					ThreadUtil.milliSleep(3000);
				}
			}
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void turnOffUvLed(ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/box/uv_led/power", "0");
			ThreadUtil.milliSleep(1000);
			if (MQTT.stateBoxUvLed.equals("0")) {
				_log.info("Successful turning off the UV led");
			} else {
				_log.info("Unsuccessful turning off the UV led");
				while (!MQTT.stateBoxUvLed.equals("0")) {
					MQTT.publish("data/box/uv_led/power", "0");
					_log.info("Trying to turn off the UV led ...");
					ThreadUtil.milliSleep(3000);
				}
			}
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
		
		
	public void turnOnStirrer(ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/box/stirrer/power", "1");
			ThreadUtil.milliSleep(1000);
			if (MQTT.stateBoxStirrer.equals("1")) {
				_log.info("Successful turning on the magnetic stirrer");
			} else {
				_log.info("Unsuccessful turning on the magnetic stirrer");
				while (!MQTT.stateBoxStirrer.equals("1")) {
					MQTT.publish("data/box/stirrer/power", "1");
					_log.info("Trying turn on the magnetic stirrer ...");
					ThreadUtil.milliSleep(3000);
				}
			}
		} catch (Exception e) {
			_log.info(e.getMessage());
			getAplControl.pause();
			_log.info("Program paused");
		}
	}
	
	
	public void turnOffStirrer(ITaskLogger _log, IApplicationControl getAplControl) {
		try {
			MQTT.publish("data/box/stirrer/power", "0");
			ThreadUtil.milliSleep(1000);
			if (MQTT.stateBoxStirrer.equals("0")) {
				_log.info("Successful turning off the magnetic stirrer");
			} else {
				_log.info("Unsuccessful turning off the magnetic stirrer");
				while (!MQTT.stateBoxStirrer.equals("0")) {
					MQTT.publish("data/box/stirrer/power", "0");
					_log.info("Trying turn off the magnetic stirrer ...");
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
