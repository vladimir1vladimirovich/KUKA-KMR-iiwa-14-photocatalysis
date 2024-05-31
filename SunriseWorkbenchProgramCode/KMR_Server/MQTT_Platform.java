package ItmoInfochemLab;

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


public class MQTT_Platform {
	@Inject
	public static ITaskLogger _log;
	static MqttClient client;
    static String brokerUrl = "tcp://172.31.1.207:1883";
    static String username = "broker_name";
    static String password = "broker_password";
    static String data_str;
    static int data_int;
    static float[] cam_coordinates = new float[6];
    static float[] transform_coordinates = new float[6];
    
    
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
                if (topic.contains("state/coordsFromCamera")) {
                    if (data_str.equals("Marker coordinates with given ID not found")) {
                    	for (byte i=0; i<6; i++) {
	                        cam_coordinates[i] = 0.0f;
	                    }
                    	_log.info("Marker coordinates with given ID not found");
                    } else {
	                	data_str = data_str.substring(1, data_str.length()-1);
	                    String[] cam_coordinates_str = data_str.split(", ");
	                    for (byte i=0; i<6; i++) {
	                        cam_coordinates[i] = Float.parseFloat(cam_coordinates_str[i]);
	                    }
                    }
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
    
    
    public static double[] getTableMarkerFrame(int id, int marker_size, int desired_X, int desired_Z) throws MqttException {
    	double[] markerId_and_markerSize = new double[] {id, marker_size};
		MQTT_Platform.publish("data/coordsFromCamera", Arrays.toString(markerId_and_markerSize));
		ThreadUtil.milliSleep(1500);
		
		while (data_str.equals("Marker coordinates with given ID not found")) {
			MQTT_Platform.publish("data/coordsFromCamera", Arrays.toString(markerId_and_markerSize));
			ThreadUtil.milliSleep(2000);
			System.out.println("Send request again ...");
		}

		double x = MQTT_Platform.cam_coordinates[0];
		double y = MQTT_Platform.cam_coordinates[2];
		double theta = MQTT_Platform.cam_coordinates[4];
	
		double half_platform = 217;
		double dy_cam = 4;
		double dz_cam = 7;
		if (x < 0) {
			dy_cam = - dy_cam;
		}
		double dX = (x + dy_cam - desired_X) / 1000.0;
		double dY = - (y - dz_cam - half_platform - desired_Z) / 1000.0;
		double dTheta = - theta;
		
		double[] platform_coords = {dX, dY, dTheta};
		
		return platform_coords;
//		_log.info("dX: " + String.valueOf(dX) + " [m]");
//		_log.info("dY: " + String.valueOf(dY) + " [m]");
//		_log.info("dTheta: " + String.valueOf(dTheta) + " [rad]");
	}
}
