package photocatalysis;

import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.task.ITaskLogger;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import javax.inject.Inject;


public class MQTT {
    public static MqttClient client;
    private static String brokerUrl = "tcp://172.31.1.207:1883";
    private static String username = "broker_name";
    private static String password = "broker_password";
    public static String data_str;
    public static int data_gripepr_angle;
    public static String stateLockStr;
    public static String stateRegulatorStr;
    public static String stateFumeCupboardDoor;
    public static String stateBoxInitialization;
    public static String stateBoxFrontDoor;
    public static String stateBoxUpperDoor;
    public static String stateBoxStirrer;
    public static String stateBoxUvLed;
    public static float[] cam_coordinates = new float[6];
    public static float[] transform_coordinates = new float[6];
    public static String stateClampStr;
    public static int data_clamp_compress = 0;
    public static double data_clamp_rotate_clockwise = 0.0;
    public static double data_clamp_rotate_counterclockwise = 0.0;
    public static String stateVolumeChangerStr;
    public static int data_volume_changer_compress = 0;
    public static double data_volume_changer_rotate_clockwise = 0.0;
    public static double data_volume_changer_rotate_counterclockwise = 0.0;
    
    
    public static void initializeMQTT(String clientId) throws MqttException {
    	client = new MqttClient(brokerUrl, clientId);
        MqttConnectOptions mqttConnectOptions = new MqttConnectOptions();
        mqttConnectOptions.setUserName(username);
        mqttConnectOptions.setPassword(password.toCharArray());
        client.connect(mqttConnectOptions);
        client.setCallback(new MqttCallback() {
            @Override
            public void connectionLost(Throwable cause) {
                System.out.println("client lost connection " + cause);
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) {
                data_str = new String(message.getPayload());

                if (topic.contains("state/gripper/rotate")) {
                	data_str = data_str.substring(9, data_str.length());
                	data_gripepr_angle = Integer.parseInt(data_str);
                }
                if (topic.contains("state/transformCoords")) {
                	if (data_str.equals("Marker coordinates with given ID not found")) {
                		for (byte i=0; i<6; i++) {
                			transform_coordinates[i] = 0.0f;
	                    }
                		System.out.println("Marker coordinates with given ID not found");
                	} else {
	                	data_str = data_str.substring(1, data_str.length()-1);
	                    String[] transform_coordinates_str = data_str.split(", ");
	                    for (byte i=0; i<6; i++) {
	                    	transform_coordinates[i] = Float.parseFloat(transform_coordinates_str[i]);
	                    }
                	}
                }
                
                if (topic.equals("state/clamp/compress")) {
                	if (data_str.contains("Compress to")) {
                		data_str = data_str.substring(12);
                        data_clamp_compress = Integer.parseInt(data_str);
                	}
                }
            	if (topic.equals("state/clamp/rotate_clockwise")) {
                	if (data_str.contains("rotate clockwise")) {
                		stateClampStr = data_str;
                		data_str = data_str.substring(26);
                		data_clamp_rotate_clockwise = Double.parseDouble(data_str);
                	}
            	}
            	if (topic.equals("state/clamp/rotate_counterclockwise")) {
                	if (data_str.contains("rotate counterclockwise")) {
                		stateClampStr = data_str;
                		data_str = data_str.substring(33);
                		data_clamp_rotate_counterclockwise = Double.parseDouble(data_str);
                    }
            	}
            	
                if (topic.equals("state/volume_changing_device/compress")) {
                	if (data_str.contains("Compress to")) {
                		data_str = data_str.substring(12);
                		data_volume_changer_compress = Integer.parseInt(data_str);
                	}
                }
            	if (topic.equals("state/volume_changing_device/rotate_clockwise")) {
                	if (data_str.contains("rotate clockwise")) {
                		stateVolumeChangerStr = data_str;
                		data_str = data_str.substring(26);
                		data_volume_changer_rotate_clockwise = Double.parseDouble(data_str);
                	}
            	}
            	if (topic.equals("state/volume_changing_device/rotate_counterclockwise")) {
                	if (data_str.contains("rotate counterclockwise")) {
                		stateVolumeChangerStr = data_str;
                		data_str = data_str.substring(33);
                		data_volume_changer_rotate_counterclockwise = Double.parseDouble(data_str);
                    }
            	}
                
                if (topic.equals("state/muffle")) {
                	stateLockStr = data_str;
                }
                if (topic.equals("state/muffle_relay")) {
                	stateRegulatorStr = data_str;
                }
                if (topic.equals("state/fumeCupboardDoor")) {
                	stateFumeCupboardDoor = data_str;
                }

                if (topic.equals("state/box/initialize")) {
                	stateBoxInitialization = data_str;
                }
                if (topic.equals("state/box/front_door")) {
                	stateBoxFrontDoor = data_str; //"open", "close"
                }
                if (topic.equals("state/box/upper_door")) {
                	stateBoxUpperDoor = data_str; //"open", "close"
                }
                if (topic.equals("state/box/stirrer/power")) {
                	stateBoxStirrer = data_str; //"0", "1"
                }
                if (topic.equals("state/box/uv_led/power")) {
                	stateBoxUvLed = data_str; //"0", "1"
                }
                
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {
                //TO DO
            }
        });
    }

    public static void publish(String publish_topic, String msg) throws MqttException {
        client.publish(publish_topic, msg.getBytes(), 2, false);
    }

    public static void subscribe(String subscribe_topic) throws MqttException {
        client.subscribe(subscribe_topic, 2);
    }

    public static void close() throws MqttException{
        client.disconnect();
    }

    public static Frame getMarkerFrame(int id, int marker_size, LBR iiwa) throws MqttException {
		Frame frame = null;
    	double x_flange = iiwa.getFlange().getX();
		double y_flange = iiwa.getFlange().getY();
		double z_flange = iiwa.getFlange().getZ();
		double a_flange = iiwa.getFlange().getAlphaRad();
		double b_flange = iiwa.getFlange().getBetaRad();
		double c_flange = iiwa.getFlange().getGammaRad();
		double[] flangeAndPointCoords = new double[] {x_flange, y_flange, z_flange, a_flange, b_flange, c_flange, id, marker_size};
		MQTT.publish("data/transformCoords", Arrays.toString(flangeAndPointCoords));
		ThreadUtil.milliSleep(1500);

		while (data_str.equals("Marker coordinates with given ID not found")) {
			MQTT.publish("data/transformCoords", Arrays.toString(flangeAndPointCoords));
			ThreadUtil.milliSleep(2000);
			System.out.println("Send request again ...");
		}
		
		frame = new Frame(MQTT.transform_coordinates[0], MQTT.transform_coordinates[1], MQTT.transform_coordinates[2], MQTT.transform_coordinates[3], MQTT.transform_coordinates[4], MQTT.transform_coordinates[5]);
		return frame;
	}
    
    
}
